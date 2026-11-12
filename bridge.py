#!/usr/bin/env python3
# Arduino UNO Q MPU-to-MCU HTTP/MessagePack bridge  
# Roni Bandini 11/2026
# Note: this is called from potSliderSorter.py

import json
import socket
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import msgpack

SOCKET_PATH = "/var/run/arduino-router.sock"
HTTP_HOST = "127.0.0.1"
HTTP_PORT = 8080
CALL_TIMEOUT = 20.0


def log(message):
    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    print(f"[{timestamp}] {message}", flush=True)


def makeJsonSafe(value):
    """Convert MessagePack byte strings recursively before JSON encoding.

    Arduino String return values can arrive from RouterBridge as bytes. A raw
    bytes object cannot be passed to json.dumps(), which was the failure in the
    unmodified Jaime HTTP handler for methods returning Arduino String values.
    """
    if isinstance(value, (bytes, bytearray, memoryview)):
        return bytes(value).decode("utf-8", errors="replace")
    if isinstance(value, dict):
        return {makeJsonSafe(key): makeJsonSafe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [makeJsonSafe(item) for item in value]
    return value


class ArduinoBridge:
    """Persistent MessagePack-RPC connection to the UNO Q Arduino Router."""

    def __init__(self):
        self.sock = None
        self.running = False
        self.counter = 0
        self.condition = threading.Condition()
        self.pending = {}
        self.callLock = threading.Lock()
        self.sendLock = threading.Lock()

    def connect(self):
        if self.running and self.sock is not None:
            return True

        bridgeSocket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            bridgeSocket.connect(SOCKET_PATH)
        except OSError as error:
            bridgeSocket.close()
            log(f"Cannot connect to {SOCKET_PATH}: {error}")
            return False

        self.sock = bridgeSocket
        self.running = True
        receiver = threading.Thread(target=self.receiveLoop, daemon=True)
        receiver.start()
        log(f"Connected to {SOCKET_PATH}")
        return True

    def call(self, methodName, *args):
        """Call an MCU RPC method."""
        with self.callLock:
            if not self.running or self.sock is None:
                return {
                    "success": False,
                    "result": None,
                    "error": "Arduino Router is disconnected",
                }

            with self.condition:
                self.counter += 1
                messageId = self.counter
                self.pending[messageId] = {
                    "success": False,
                    "result": None,
                    "error": "",
                    "complete": False,
                }

            payload = msgpack.packb([0, messageId, methodName, list(args)], use_bin_type=True)
            try:
                with self.sendLock:
                    self.sock.sendall(payload)
                log(f"CALL {methodName}({', '.join(str(arg) for arg in args)})")
            except OSError as error:
                with self.condition:
                    self.pending.pop(messageId, None)
                self.markDisconnected(str(error))
                return {"success": False, "result": None, "error": str(error)}

            with self.condition:
                completed = self.condition.wait_for(
                    lambda: messageId not in self.pending or self.pending[messageId]["complete"],
                    timeout=CALL_TIMEOUT,
                )
                response = self.pending.pop(
                    messageId,
                    {"success": False, "result": None, "error": "MCU RPC timeout"},
                )

            if not completed:
                response = {"success": False, "result": None, "error": "MCU RPC timeout"}

            if response["success"]:
                log(f"MCU response {methodName}: {response['result']}")
            else:
                log(f"MCU call failed {methodName}: {response['error']}")

            return response

    def markDisconnected(self, reason):
        self.running = False
        with self.condition:
            for response in self.pending.values():
                response["error"] = reason
                response["complete"] = True
            self.condition.notify_all()
        log(f"MCU socket disconnected: {reason}")

    def receiveLoop(self):
        unpacker = msgpack.Unpacker(raw=False)
        while self.running:
            try:
                packet = self.sock.recv(4096)
                if not packet:
                    self.markDisconnected("Router closed the socket")
                    break

                unpacker.feed(packet)
                for message in unpacker:
                    if not isinstance(message, list) or len(message) != 4:
                        continue

                    messageType, messageId, methodOrError, result = message

                    # MessagePack-RPC response: [1, id, error, result].
                    if messageType == 1:
                        with self.condition:
                            response = self.pending.get(messageId)
                            if response is not None:
                                if methodOrError is None:
                                    response["success"] = True
                                    response["result"] = result
                                else:
                                    response["success"] = False
                                    response["error"] = str(methodOrError)
                                response["complete"] = True
                                self.condition.notify_all()

                    # Acknowledge MCU-to-MPU notifications, as Jaime's bridge does.
                    elif messageType == 0:
                        acknowledgement = msgpack.packb(
                            [1, messageId, None, "ok"],
                            use_bin_type=True,
                        )
                        with self.sendLock:
                            self.sock.sendall(acknowledgement)

            except (OSError, ValueError, msgpack.UnpackException) as error:
                if self.running:
                    self.markDisconnected(str(error))
                break

    def disconnect(self):
        self.running = False
        with self.condition:
            for response in self.pending.values():
                response["error"] = "Bridge is shutting down"
                response["complete"] = True
            self.condition.notify_all()

        try:
            if self.sock is not None:
                self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            if self.sock is not None:
                self.sock.close()
        except OSError:
            pass
        self.sock = None


bridge = ArduinoBridge()


class RequestHandler(BaseHTTPRequestHandler):
    def log_message(self, formatString, *args):
        return

    def sendJson(self, statusCode, payload):
        safePayload = makeJsonSafe(payload)
        body = json.dumps(safePayload, ensure_ascii=False).encode("utf-8")
        self.send_response(statusCode)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/health":
            self.sendJson(200, {"status": "ok", "connected": bridge.running})
        else:
            self.sendJson(404, {"success": False, "error": "not found"})

    def do_POST(self):
        if self.path != "/command":
            self.sendJson(404, {"success": False, "error": "not found"})
            return

        try:
            contentLength = int(self.headers.get("Content-Length", "0"))
            if contentLength <= 0 or contentLength > 65536:
                self.sendJson(400, {"success": False, "result": None, "error": "invalid request size"})
                return

            requestData = json.loads(self.rfile.read(contentLength).decode("utf-8"))
            methodName = requestData.get("method")
            args = requestData.get("args", [])
            if not isinstance(methodName, str) or not methodName:
                raise ValueError("method must be a non-empty string")
            if not isinstance(args, list):
                raise ValueError("args must be a list")

            log(f"HTTP command: {methodName} {args}")
            response = bridge.call(methodName, *args)
            # Critical fix: decode any bytes in the RPC result before JSON output.
            self.sendJson(200, response)

        except (UnicodeDecodeError, json.JSONDecodeError, KeyError, TypeError, ValueError) as error:
            log(f"Invalid HTTP request: {error}")
            self.sendJson(400, {"success": False, "result": None, "error": str(error)})
        except Exception as error:
            log(f"HTTP request error: {error}")
            self.sendJson(500, {"success": False, "result": None, "error": str(error)})


def main():
    log("Starting Conveyor Line Object Detection and Sorter bridge")
    if not bridge.connect():
        log("FATAL: Arduino Router socket is unavailable")
        log("Upload potSliderSorter.ino and confirm the Arduino Router service is running")
        return 1

    try:
        server = ThreadingHTTPServer((HTTP_HOST, HTTP_PORT), RequestHandler)
    except OSError as error:
        bridge.disconnect()
        log(f"Cannot bind HTTP {HTTP_HOST}:{HTTP_PORT}: {error}")
        log("Another bridge may already be running. Run only one bridge server on port 8080")
        return 1

    log(f"HTTP API listening on http://{HTTP_HOST}:{HTTP_PORT}")
    log("UNO Q sorter bridge ready")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        log("Stopping bridge")
    finally:
        server.server_close()
        bridge.disconnect()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
