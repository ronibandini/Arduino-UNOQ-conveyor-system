#!/usr/bin/env python3

# Conveyor Line Object Detection and Sorter 
# Edge Impulse + Arduino UNO Q
# Roni Bandini, 11/2026 GPL License

import ast
import json
import os
import queue
import re
import select
import subprocess
import sys
import threading
import time
from collections import deque
from datetime import datetime, timezone
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

try:
    import termios
    import tty
except ImportError:
    termios = None
    tty = None

try:
    import paho.mqtt.client as mqtt
except ImportError:
    mqtt = None

# -----------------------------------------------------------------------------
# Local configuration: edit these values in this file.
# -----------------------------------------------------------------------------

runnerCommand = ["edge-impulse-linux-runner"]
bridgeUrl = "http://127.0.0.1:8080"
bridgeStartupTimeoutSeconds = 12.0

potLabel = "pot"
sliderLabel = "slider"
classificationThreshold = 0.80
rearmMissFrames = 3
duplicateSuppressionSeconds = 3.0

# Each category has the same independent capacity in the active box.
# The line stops when POT reaches its limit OR SLIDER reaches its limit.

boxLimitPerCategory = 4
potBoxLimit = boxLimitPerCategory
sliderBoxLimit = boxLimitPerCategory

statusPollSeconds = 0.35
dashboardRefreshSeconds = 0.20
bridgeTimeoutSeconds = 4.0
unsHeartbeatSeconds = 2.0
boxFullStopDelaySeconds = 3.0   

# MQTT / Unified Namespace configuration.

mqttEnabled = True
mqttHost = "127.0.0.1"
mqttPort = 1883
mqttClientId = "unoq-pot-slider-sorter"
mqttUsername = ""
mqttPassword = ""
mqttTls = False
mqttKeepalive = 30
mqttQos = 1

# ISA-95 hierarchy. Edit these values for the installation.

isa95Hierarchy = {
    "enterprise": "demo-enterprise",
    "site": "site-01",
    "area": "area-01",
    "line": "line-01",
    "cell": "cell-01",
    "equipment": "uno-q-pot-slider-sorter",
}

if duplicateSuppressionSeconds < 0:
    raise ValueError("duplicateSuppressionSeconds cannot be negative")
if potBoxLimit < 1 or sliderBoxLimit < 1:
    raise ValueError("Category box limits must be at least 1")


def topicSegment(value):
    cleanValue = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(value).strip())
    return cleanValue or "unknown"


unsRootTopic = "/".join(topicSegment(value) for value in isa95Hierarchy.values())
unsStateTopic = f"{unsRootTopic}/state"
unsAvailabilityTopic = f"{unsRootTopic}/availability"
unsDetectionTopic = f"{unsRootTopic}/events/detection"
unsObjectTopic = f"{unsRootTopic}/events/object"
unsSorterTopic = f"{unsRootTopic}/events/sorter"
unsBoxTopic = f"{unsRootTopic}/events/box"

# -----------------------------------------------------------------------------
# Edge Impulse output parsing
# -----------------------------------------------------------------------------

inferencePattern = re.compile(
    r"(?:classifyRes|boundingBoxes)\s+([0-9]+(?:\.[0-9]+)?)\s*ms",
    re.IGNORECASE,
)


def parsePayload(line):
    """Decode the JSON/Python-literal payload printed by edge-impulse-linux-runner."""
    lowerLine = line.lower()
    markerPositions = [
        position for marker in ("boundingboxes", "classifyres")
        if (position := lowerLine.find(marker)) >= 0
    ]
    if not markerPositions:
        return None

    markerPosition = min(markerPositions)
    arrayStart = line.find("[", markerPosition)
    objectStart = line.find("{", markerPosition)

    if arrayStart >= 0 and (objectStart < 0 or arrayStart < objectStart):
        payloadText = line[arrayStart:line.rfind("]") + 1]
    elif objectStart >= 0:
        payloadText = line[objectStart:line.rfind("}") + 1]
    else:
        return None

    if not payloadText or payloadText[-1] not in "]}":
        return None

    try:
        return json.loads(payloadText)
    except json.JSONDecodeError:
        try:
            return ast.literal_eval(payloadText)
        except (ValueError, SyntaxError):
            return None


def normalizeScore(rawScore):
    try:
        score = float(rawScore)
    except (TypeError, ValueError):
        return None

    if 1.0 < score <= 100.0:
        score /= 100.0
    return max(0.0, min(1.0, score))


def parseBoundingBoxes(boxes):
    """Return maximum confidence per class from an Edge Impulse box list."""
    if not isinstance(boxes, list):
        return None

    scores = {}
    for box in boxes:
        if not isinstance(box, dict):
            continue
        label = box.get("label")
        if label is None:
            continue
        score = normalizeScore(
            box.get("value", box.get("confidence", box.get("score")))
        )
        if score is None:
            continue
        normalizedLabel = str(label).strip().lower()
        if normalizedLabel:
            scores[normalizedLabel] = max(
                scores.get(normalizedLabel, 0.0), score
            )
    return scores


def parseClassification(line):
    """Parse classifyRes dictionaries and FOMO/boundingBoxes arrays."""
    result = parsePayload(line)
    if result is None:
        return None

    if isinstance(result, list):
        return parseBoundingBoxes(result)
    if not isinstance(result, dict):
        return None

    for wrapperKey in ("result", "results", "output"):
        wrapped = result.get(wrapperKey)
        if isinstance(wrapped, (dict, list)):
            result = wrapped
            break

    if isinstance(result, list):
        return parseBoundingBoxes(result)
    if not isinstance(result, dict):
        return None

    for boxesKey in ("boundingBoxes", "bounding_boxes", "boundingboxes"):
        if boxesKey in result:
            return parseBoundingBoxes(result[boxesKey])

    for classificationKey in ("classification", "classes", "scores"):
        classification = result.get(classificationKey)
        if isinstance(classification, dict):
            result = classification
            break

    scores = {}
    for label, rawScore in result.items():
        score = normalizeScore(rawScore)
        if score is not None:
            scores[str(label).strip().lower()] = score
    return scores if scores else None


def parseInferenceMs(line):
    match = inferencePattern.search(line)
    return float(match.group(1)) if match else None


def readRunnerOutput(process, outputQueue):
    """Read runner stdout asynchronously to keep the dashboard responsive."""
    try:
        for line in process.stdout:
            try:
                outputQueue.put_nowait(line.rstrip("\r\n"))
            except queue.Full:
                try:
                    outputQueue.get_nowait()
                except queue.Empty:
                    pass
                try:
                    outputQueue.put_nowait(line.rstrip("\r\n"))
                except queue.Full:
                    pass
    finally:
        try:
            outputQueue.put_nowait(None)
        except queue.Full:
            pass

# -----------------------------------------------------------------------------
# HTTP bridge client
# -----------------------------------------------------------------------------


def bridgeGet(path):
    request = Request(bridgeUrl + path, method="GET")
    with urlopen(request, timeout=bridgeTimeoutSeconds) as response:
        return json.loads(response.read().decode("utf-8"))


def bridgeCall(methodName, *args):
    payload = json.dumps({"method": methodName, "args": list(args)}).encode("utf-8")
    request = Request(
        bridgeUrl + "/command",
        data=payload,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urlopen(request, timeout=bridgeTimeoutSeconds) as response:
        result = json.loads(response.read().decode("utf-8"))
    if not result.get("success"):
        raise RuntimeError(result.get("error", f"MCU call {methodName} failed"))
    return result.get("result")


def getBridgeHealth():
    """Return bridge health, or None when the local HTTP server is unavailable."""
    try:
        return bridgeGet("/health")
    except (URLError, HTTPError, TimeoutError, OSError, ValueError):
        return None


def checkBridge():
    health = getBridgeHealth()
    return bool(
        health
        and health.get("status") == "ok"
        and health.get("connected", True)
    )


def readBridgeLogTail(logPath, maxLines=20):
    try:
        with open(logPath, "r", encoding="utf-8", errors="replace") as logFile:
            return "\n".join(logFile.readlines()[-maxLines:]).strip()
    except OSError:
        return "Bridge log is not available."


def ensureBridgeRunning(startupTimeoutSeconds=12.0):
    """Use an existing bridge or launch bridge.py detached in the background.

    The Arduino Router socket is commonly restricted to root or the
    arduino-router group. If the current user cannot write to the socket,
    sudo authentication is requested once and the bridge is launched via sudo.
    """
    health = getBridgeHealth()
    if health is not None:
        if health.get("status") == "ok" and health.get("connected", True):
            print("UNO Q bridge already running.")
            return True
        if health.get("status") == "ok" and not health.get("connected", False):
            raise RuntimeError(
                "The HTTP bridge is already running, but its Arduino Router "
                "socket is disconnected. Restart the existing bridge after "
                "the MCU firmware is running; a second server cannot use port 8080."
            )

    scriptDirectory = os.path.dirname(os.path.abspath(__file__))
    bridgePath = os.path.join(scriptDirectory, "bridge.py")
    if not os.path.isfile(bridgePath):
        raise RuntimeError(
            f"bridge.py was not found next to potSliderSorter.py: {bridgePath}"
        )

    logPath = os.path.join(scriptDirectory, "bridge.log")
    command = [sys.executable, bridgePath]

    socketPath = "/var/run/arduino-router.sock"
    if os.geteuid() != 0 and os.path.exists(socketPath) and not os.access(socketPath, os.W_OK):
        print("Arduino Router socket requires elevated permissions.")
        print("Requesting sudo authentication to start the bridge in the background...")
        try:
            subprocess.run(["sudo", "-v"], check=True)
        except (OSError, subprocess.CalledProcessError) as error:
            raise RuntimeError(
                "Cannot authenticate sudo to access /var/run/arduino-router.sock. "
                "Add your user to the arduino-router group or allow sudo for the bridge."
            ) from error
        command = ["sudo", "-n", sys.executable, bridgePath]

    print("Starting UNO Q bridge in the background...")
    with open(logPath, "a", encoding="utf-8", buffering=1) as logFile:
        bridgeProcess = subprocess.Popen(
            command,
            cwd=scriptDirectory,
            stdin=subprocess.DEVNULL,
            stdout=logFile,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            close_fds=True,
        )

    deadline = time.monotonic() + startupTimeoutSeconds
    while time.monotonic() < deadline:
        health = getBridgeHealth()
        if health is not None:
            if health.get("status") == "ok" and health.get("connected", True):
                print("UNO Q bridge started successfully in the background.")
                print(f"Bridge log: {logPath}")
                return True
            if health.get("status") == "ok" and not health.get("connected", False):
                if bridgeProcess.poll() is not None:
                    break
        if bridgeProcess.poll() is not None:
            break
        time.sleep(0.25)

    logDetails = readBridgeLogTail(logPath)
    raise RuntimeError(
        "Could not start a connected UNO Q bridge.\n"
        f"Bridge log ({logPath}):\n{logDetails}"
    )


def parseSortStatus(rawStatus):
    """Decode MCU CSV with independent POT and SLIDER box counters."""
    if isinstance(rawStatus, bytes):
        rawStatus = rawStatus.decode("utf-8", errors="replace")
    if not isinstance(rawStatus, str):
        return None
    parts = rawStatus.strip().split(",")
    if len(parts) != 11:
        return None
    try:
        return {
            "state": int(parts[0]),
            "partCode": int(parts[1]),
            "angle": int(parts[2]),
            "relayStopped": bool(int(parts[3])),
            "servoReady": bool(int(parts[4])),
            "detectionCode": int(parts[5]),
            "boxFull": bool(int(parts[6])),
            "potBoxCount": int(parts[7]),
            "sliderBoxCount": int(parts[8]),
            "boxNumber": int(parts[9]),
            "inferenceReady": bool(int(parts[10])),
        }
    except ValueError:
        return None


# -----------------------------------------------------------------------------
# Keyboard acknowledgement for installing a new box
# -----------------------------------------------------------------------------


class OperatorKeyReader:
    """Read one key without blocking the dashboard; restore terminal on exit."""

    def __init__(self):
        self.fd = None
        self.originalAttributes = None
        if sys.stdin.isatty() and termios is not None and tty is not None:
            self.fd = sys.stdin.fileno()
            self.originalAttributes = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
            currentAttributes = termios.tcgetattr(self.fd)
            currentAttributes[3] &= ~termios.ECHO
            termios.tcsetattr(self.fd, termios.TCSANOW, currentAttributes)

    def readKey(self):
        if self.fd is None:
            return None
        readable, _, _ = select.select([self.fd], [], [], 0)
        if not readable:
            return None
        try:
            return os.read(self.fd, 1).decode("utf-8", errors="ignore")
        except OSError:
            return None

    def close(self):
        if self.fd is not None and self.originalAttributes is not None:
            termios.tcsetattr(self.fd, termios.TCSANOW, self.originalAttributes)
            self.fd = None

# -----------------------------------------------------------------------------
# MQTT publisher: retained state, event topics, availability and LWT
# -----------------------------------------------------------------------------


class UnsPublisher:
    """Publish sorter state and events under an ISA-95 MQTT topic hierarchy."""

    def __init__(self):
        if mqtt is None:
            raise RuntimeError(
                "paho-mqtt is not installed. Install it with: "
                "sudo apt install python3-paho-mqtt"
            )

        self.client = mqtt.Client(client_id=mqttClientId, clean_session=True)
        if mqttUsername:
            self.client.username_pw_set(mqttUsername, mqttPassword or None)
        if mqttTls:
            self.client.tls_set()

        self.lock = threading.Lock()
        self.connected = False
        self.latestState = None
        self.pendingEvents = deque(maxlen=100)
        self.client.on_connect = self.onConnect
        self.client.on_disconnect = self.onDisconnect
        if hasattr(self.client, "on_connect_fail"):
            self.client.on_connect_fail = self.onConnectFail
        self.client.reconnect_delay_set(min_delay=1, max_delay=30)

        willPayload = self.jsonPayload({
            "status": "offline",
            "timestamp": self.timestamp(),
            "equipment": isa95Hierarchy["equipment"],
        })
        self.client.will_set(
            unsAvailabilityTopic,
            payload=willPayload,
            qos=mqttQos,
            retain=True,
        )
        self.client.connect_async(mqttHost, mqttPort, keepalive=mqttKeepalive)
        self.client.loop_start()

    @staticmethod
    def timestamp():
        return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")

    @staticmethod
    def jsonPayload(payload):
        return json.dumps(payload, separators=(",", ":"), ensure_ascii=False)

    def onConnect(self, client, userdata, flags, resultCode):
        try:
            isConnected = int(resultCode) == 0
        except (TypeError, ValueError):
            isConnected = str(resultCode).lower() in {"success", "success (0)"}
        with self.lock:
            self.connected = isConnected
            latestState = self.latestState
            queuedEvents = list(self.pendingEvents) if isConnected else []
            if isConnected:
                self.pendingEvents.clear()
        if not isConnected:
            print(f"[UNS] MQTT connection refused: {resultCode}", flush=True)
            return
        print(f"[UNS] Connected to MQTT broker {mqttHost}:{mqttPort}", flush=True)
        self.publishAvailability("online")
        if latestState:
            self.publishRaw(unsStateTopic, latestState, retain=True)
        for eventTopic, eventPayload in queuedEvents:
            self.publishRaw(eventTopic, eventPayload, retain=False)

    def onDisconnect(self, client, userdata, resultCode):
        with self.lock:
            self.connected = False
        print(f"[UNS] MQTT disconnected ({resultCode}); reconnecting", flush=True)

    def onConnectFail(self, client, userdata):
        with self.lock:
            self.connected = False
        print("[UNS] MQTT broker unavailable; Paho will retry", flush=True)

    def publishRaw(self, topic, payload, retain=False):
        try:
            result = self.client.publish(topic, payload, qos=mqttQos, retain=retain)
            if result.rc != mqtt.MQTT_ERR_SUCCESS:
                print(f"[UNS] Publish failed for {topic}: rc={result.rc}", flush=True)
                return False
            return True
        except Exception as error:
            print(f"[UNS] Publish error for {topic}: {error}", flush=True)
            return False

    def publishAvailability(self, status):
        payload = self.jsonPayload({
            "status": status,
            "timestamp": self.timestamp(),
            "equipment": isa95Hierarchy["equipment"],
        })
        self.publishRaw(unsAvailabilityTopic, payload, retain=True)

    def updateState(self, state):
        payload = self.jsonPayload(state)
        with self.lock:
            self.latestState = payload
            isConnected = self.connected
        if isConnected:
            self.publishRaw(unsStateTopic, payload, retain=True)

    def publishEvent(self, topic, event):
        payload = self.jsonPayload(event)
        with self.lock:
            isConnected = self.connected
            if not isConnected:
                self.pendingEvents.append((topic, payload))
                return
        self.publishRaw(topic, payload, retain=False)

    def stop(self):
        with self.lock:
            isConnected = self.connected
        if isConnected:
            self.publishAvailability("offline")
            time.sleep(0.15)
        try:
            self.client.disconnect()
            self.client.loop_stop()
        except Exception:
            pass
        with self.lock:
            self.connected = False

# -----------------------------------------------------------------------------
# UNS payload builders
# -----------------------------------------------------------------------------


def buildUnsState(data):
    status = data.get("sortStatus") or {}
    stateNames = {
        0: "idle", 1: "object_registered",
        2: "box_full_final_item_transit", 3: "box_full_conveyor_stopped",
        4: "waiting_for_inference",
    }
    partNames = {0: "none", 1: "slider", 2: "pot"}
    detectionNames = {0: "none", 1: "pot", 2: "slider"}
    potBoxCount = data.get("potBoxCount", 0)
    sliderBoxCount = data.get("sliderBoxCount", 0)
    return {
        "schemaVersion": "1.0",
        "timestamp": UnsPublisher.timestamp(),
        "source": "arduino-uno-q-edge-impulse-sorter",
        "isa95": isa95Hierarchy,
        "classification": {
            "label": data.get("detection", "none"),
            "confidence": data.get("confidence", 0.0),
            "scores": data.get("scores", {}),
            "threshold": classificationThreshold,
        },
        "equipmentState": {
            "sorter": stateNames.get(status.get("state", -1), "unknown"),
            "inferenceReady": data.get("inferenceReady", False),
            "lastObject": partNames.get(status.get("partCode", 0), "unknown"),
            "conveyor": "stopped" if status.get("relayStopped", False) else "running",
            "relayStopped": status.get("relayStopped", False),
            "servoAngle": status.get("angle", 0),
            "servoReady": status.get("servoReady", False),
            "indicator": detectionNames.get(status.get("detectionCode", 0), "unknown"),
        },
        "production": {
            "potCount": data.get("potCount", 0),
            "sliderCount": data.get("sliderCount", 0),
            "recognizedObjectCount": data.get("potCount", 0) + data.get("sliderCount", 0),
        },
        "packaging": {
            "activeBoxNumber": data.get("boxNumber", 1),
            "potUnitsInCurrentBox": potBoxCount,
            "sliderUnitsInCurrentBox": sliderBoxCount,
            "potBoxLimit": potBoxLimit,
            "sliderBoxLimit": sliderBoxLimit,
            "potFillPercent": min(100.0, potBoxCount * 100.0 / potBoxLimit),
            "sliderFillPercent": min(100.0, sliderBoxCount * 100.0 / sliderBoxLimit),
            "boxFull": data.get("boxFull", False),
            "boxFullCategory": data.get("boxFullCategory", "none"),
            "boxStopPending": data.get("boxFull", False) and not data.get("boxStopConfirmed", False),
            "boxStopDelaySeconds": boxFullStopDelaySeconds,
            "completedBoxes": data.get("boxesCompleted", 0),
        },
        "detectionPolicy": {
            "confidenceThreshold": classificationThreshold,
            "duplicateSuppressionSeconds": duplicateSuppressionSeconds,
            "rearmMissFrames": rearmMissFrames,
        },
        "performance": {
            "inferenceMs": data.get("inferenceMs"),
            "frameCount": data.get("frames", 0),
            "uptimeSeconds": int(time.monotonic() - data.get("startedAt", time.monotonic())),
        },
        "lastEvent": data.get("lastEvent", ""),
    }


def buildUnsEvent(eventType, data, details):
    return {
        "schemaVersion": "1.0",
        "timestamp": UnsPublisher.timestamp(),
        "eventType": eventType,
        "source": "arduino-uno-q-edge-impulse-sorter",
        "isa95": isa95Hierarchy,
        "equipment": isa95Hierarchy["equipment"],
        "box": {
            "number": data.get("boxNumber", 1),
            "potUnits": data.get("potBoxCount", 0),
            "sliderUnits": data.get("sliderBoxCount", 0),
            "potLimit": potBoxLimit,
            "sliderLimit": sliderBoxLimit,
            "full": data.get("boxFull", False),
            "fullCategory": data.get("boxFullCategory", "none"),
            "completed": data.get("boxesCompleted", 0),
        },
        "details": details,
        "counts": {
            "pot": data.get("potCount", 0),
            "slider": data.get("sliderCount", 0),
        },
    }


# -----------------------------------------------------------------------------
# ASCII dashboard
# -----------------------------------------------------------------------------


BIG_DIGITS = {
    "0": [" ### ", "#   #", "#   #", "#   #", " ### "],
    "1": ["  #  ", " ##  ", "  #  ", "  #  ", " ### "],
    "2": [" ### ", "#   #", "   # ", "  #  ", "#####"],
    "3": ["#### ", "    #", " ### ", "    #", "#### "],
    "4": ["#  # ", "#  # ", "#####", "   # ", "   # "],
    "5": ["#####", "#    ", "#### ", "    #", "#### "],
    "6": [" ### ", "#    ", "#### ", "#   #", " ### "],
    "7": ["#####", "    #", "   # ", "  #  ", "  #  "],
    "8": [" ### ", "#   #", " ### ", "#   #", " ### "],
    "9": [" ### ", "#   #", " ####", "    #", " ### "],
}

stateNames = {
    0: "IDLE",
    1: "OBJECT REGISTERED",
    2: "BOX FULL / FINAL ITEM IN TRANSIT",
    3: "BOX FULL / CONVEYOR STOPPED",
    4: "WAITING FOR INFERENCE",
}


def scoreBar(score, width=15):
    score = max(0.0, min(1.0, score))
    filled = int(round(score * width))
    return "[" + "#" * filled + "." * (width - filled) + "]"


def bigNumber(value):
    digits = str(max(0, int(value))).zfill(4)[-4:]
    return ["  ".join(BIG_DIGITS[digit][row] for digit in digits) for row in range(5)]


def fitText(value, width):
    value = str(value)
    if len(value) > width:
        return value[:max(0, width - 3)] + "..."
    return value.ljust(width)


def percentageBar(count, limit, width=22):
    percentage = min(100.0, max(0.0, count * 100.0 / limit))
    filled = int(round(percentage * width / 100.0))
    return f"[{('#' * filled)}{('.' * (width - filled))}] {percentage:5.1f}%"


def renderDashboard(data, mqttConnected):
    width = 103
    colWidth = 33
    border = "+" + "-" * (width - 2) + "+"
    status = data.get("sortStatus") or {}
    potScore = data["scores"].get(potLabel, 0.0)
    sliderScore = data["scores"].get(sliderLabel, 0.0)
    stateName = stateNames.get(status.get("state", -1), "UNKNOWN")
    objectName = {0: "NONE", 1: "SLIDER", 2: "POT"}.get(status.get("partCode", 0), "UNKNOWN")
    ledName = {0: "GREEN / NONE", 1: "YELLOW / POT", 2: "RED / SLIDER"}.get(
        status.get("detectionCode", 0), "UNKNOWN"
    )
    conveyorState = "STOPPED" if status.get("relayStopped", False) else "RUNNING"
    if data["boxFull"]:
        ledName = "RED FLASHING" if data["boxStopConfirmed"] else "FINAL ITEM IN TRANSIT"
    servoState = f"{status.get('angle', 0)} DEG" if status.get("servoReady", False) else "NOT READY"
    inferenceText = "-- ms" if data["inferenceMs"] is None else f"{data['inferenceMs']:.1f} ms"
    elapsedSeconds = int(time.monotonic() - data["startedAt"])
    elapsedText = time.strftime("%H:%M:%S", time.gmtime(elapsedSeconds))
    cooldownRemaining = max(0.0, data.get("duplicateBlockedUntil", 0.0) - time.monotonic())

    left = [
        "CLASSIFICATION / CURRENT FRAME",
        f"POT    {scoreBar(potScore)} {potScore * 100:5.1f}%",
        f"SLIDER {scoreBar(sliderScore)} {sliderScore * 100:5.1f}%",
        f"THRESHOLD       {classificationThreshold * 100:.0f}%",
    ]
    middle = [
        "MCU / CONVEYOR STATUS",
        f"BRIDGE       {'ONLINE' if data['bridgeOnline'] else 'OFFLINE'}",
        f"INFERENCE    {'READY' if data['inferenceReady'] else 'WAITING'}",
        f"CONVEYOR     {conveyorState}",
        f"SORT STATE   {stateName}",
        f"LAST OBJECT  {objectName}",
        f"SERVO        {servoState}",
        f"RGB LED      {ledName}",
    ]
    right = [
        "UNS / PRODUCTION",
        f"MQTT         {'ONLINE' if mqttConnected else 'RECONNECTING'}",
        f"BOX NUMBER   {data['boxNumber']:06d}",
        f"BOXES FULL   {data['boxesCompleted']:06d}",
        f"DUPLICATE    {cooldownRemaining:4.1f}s / {duplicateSuppressionSeconds:.1f}s",
        f"INFERENCE    {inferenceText}",
        f"FRAMES       {data['frames']:06d}",
    ]

    rows = [border]
    rows.append("|" + fitText("  CONVEYOR LINE OBJECT DETECTION AND SORTER", width - 2) + "|")
    rows.append("|" + fitText("  Edge Impulse + Arduino UNO Q", width - 2) + "|")
    rows.append("|" + fitText("  Roni Bandini  //  11/2026", width - 2) + "|")
    rows.append(border)
    for rowIndex in range(max(len(left), len(middle), len(right))):
        leftText = left[rowIndex] if rowIndex < len(left) else ""
        middleText = middle[rowIndex] if rowIndex < len(middle) else ""
        rightText = right[rowIndex] if rowIndex < len(right) else ""
        rows.append("|" + fitText(leftText, colWidth) + "|" + fitText(middleText, colWidth) + "|" + fitText(rightText, colWidth) + "|")

    rows.append(border)
    rows.append("|" + fitText("  RECOGNIZED OBJECTS  //  SESSION TOTALS", width - 2) + "|")
    rows.append("|" + fitText("  POT DETECTED", 50) + "|" + fitText("  SLIDER DETECTED", 50) + "|")
    for potRow, sliderRow in zip(bigNumber(data["potCount"]), bigNumber(data["sliderCount"])):
        rows.append("|" + fitText("  " + potRow, 50) + "|" + fitText("  " + sliderRow, 50) + "|")

    rows.append(border)
    rows.append("|" + fitText("  CURRENT BOX CAPACITY  //  INDEPENDENT CATEGORY LIMITS", width - 2) + "|")
    potProgress = percentageBar(data["potBoxCount"], potBoxLimit)
    sliderProgress = percentageBar(data["sliderBoxCount"], sliderBoxLimit)
    rows.append("|" + fitText(f"  POT    {potProgress}  {data['potBoxCount']}/{potBoxLimit}", width - 2) + "|")
    rows.append("|" + fitText(f"  SLIDER {sliderProgress}  {data['sliderBoxCount']}/{sliderBoxLimit}", width - 2) + "|")
    rows.append(border)
    rows.append("|" + fitText(
        f"  BOX {data['boxNumber']:06d}  POT {data['potBoxCount']}/{potBoxLimit}  "
        f"SLIDER {data['sliderBoxCount']}/{sliderBoxLimit}  UPTIME {elapsedText}",
        width - 2,
    ) + "|")
    rows.append("|" + fitText(f"  LAST EVENT: {data['lastEvent'] or 'NONE'}", width - 2) + "|")
    rows.append("|" + fitText(f"  UNS TOPIC: {unsRootTopic}", width - 2) + "|")
    rows.append("|" + fitText(f"  RUNNER: {data['runnerLine'] or 'Waiting for classification...'}", width - 2) + "|")
    if data["bridgeError"]:
        rows.append("|" + fitText(f"  BRIDGE ERROR: {data['bridgeError']}", width - 2) + "|")
    if data["boxFull"] and not data["boxStopConfirmed"]:
        prompt = (f"  {data['boxFullCategory'].upper()} CAPACITY REACHED - FINAL ITEM IN TRANSIT; "
                  f"CONVEYOR STOPS AFTER {boxFullStopDelaySeconds:.0f} SECONDS")
        rows.append("|" + fitText(prompt, width - 2) + "|")
    elif data["boxFull"]:
        prompt = (f"  {data['boxFullCategory'].upper()} CAPACITY REACHED - INSTALL NEW BOX, "
                  "THEN PRESS [N] OR [ENTER] TO RESUME")
        rows.append("|" + fitText(prompt, width - 2) + "|")
    else:
        rows.append("|" + fitText("  CTRL+C TO EXIT", width - 2) + "|")
    rows.append(border)

    if sys.stdout.isatty():
        sys.stdout.write("\033[2J\033[H\033[1;37m")
    sys.stdout.write("\n".join(rows) + "\n")
    if data["boxFull"] and data["boxStopConfirmed"] and sys.stdout.isatty():
        sys.stdout.write("\033[1;31m*** CATEGORY CAPACITY REACHED - CONVEYOR STOPPED - WAITING FOR OPERATOR ***\033[0m\n")
    if sys.stdout.isatty():
        sys.stdout.write("\033[0m")
    sys.stdout.flush()


# -----------------------------------------------------------------------------
# Main application
# -----------------------------------------------------------------------------


def main():
    if mqttEnabled and mqtt is None:
        print("MQTT UNS is enabled but paho-mqtt is missing.")
        print("Install it with: sudo apt install python3-paho-mqtt")
        return 1

    print("CONVEYOR LINE OBJECT DETECTION AND SORTER")
    print("Edge Impulse + Arduino UNO Q")
    print("Roni Bandini // 11/2026")
    print()
    print("Checking UNO Q bridge...")
    try:
        ensureBridgeRunning()
        initialStatus = parseSortStatus(bridgeCall("getSortStatus"))
        if initialStatus is None:
            raise RuntimeError("Invalid MCU status response")
        if not bridgeCall("waitForInference"):
            raise RuntimeError("MCU rejected waitForInference request")
        initialStatus = parseSortStatus(bridgeCall("getSortStatus"))
    except (URLError, HTTPError, TimeoutError, OSError, ValueError, RuntimeError) as error:
        print(f"Cannot communicate with the UNO Q bridge: {error}")
        return 1

    if initialStatus is None:
        print("Invalid MCU status response. Upload the updated potSliderSorter.ino.")
        return 1
    if not initialStatus["servoReady"]:
        print("MCU reports that the servo is not attached. Check SERVO_PIN.")
        return 1

    unsPublisher = None
    if mqttEnabled:
        try:
            unsPublisher = UnsPublisher()
        except Exception as error:
            print(f"Cannot start MQTT UNS publisher: {error}")
            return 1
        print(f"Starting MQTT UNS at {mqttHost}:{mqttPort}")
        print(f"ISA-95 topic root: {unsRootTopic}")

    print("Starting Edge Impulse runner...")
    try:
        runnerProcess = subprocess.Popen(
            runnerCommand,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
    except FileNotFoundError:
        if unsPublisher:
            unsPublisher.stop()
        print("edge-impulse-linux-runner was not found or is not in PATH.")
        return 1

    outputQueue = queue.Queue(maxsize=2048)
    readerThread = threading.Thread(
        target=readRunnerOutput,
        args=(runnerProcess, outputQueue),
        daemon=True,
    )
    readerThread.start()

    initialBoxNumber = max(1, initialStatus["boxNumber"])
    initialPotBoxCount = max(0, initialStatus["potBoxCount"])
    initialSliderBoxCount = max(0, initialStatus["sliderBoxCount"])
    initialBoxFull = initialStatus["boxFull"]
    initialBoxFullCategory = (
        "pot" if initialStatus["partCode"] == 2 else
        "slider" if initialStatus["partCode"] == 1 else "none"
    )
    initialBoxStopConfirmed = (
        initialBoxFull
        and initialStatus["state"] == 3
        and initialStatus["relayStopped"]
    )
    initialCompletedBoxes = max(0, initialBoxNumber - 1) + (1 if initialBoxStopConfirmed else 0)

    data = {
        "scores": {potLabel: 0.0, sliderLabel: 0.0},
        "detection": "none",
        "confidence": 0.0,
        "sortStatus": initialStatus,
        "bridgeOnline": True,
        "bridgeError": "",
        "inferenceReady": initialStatus["inferenceReady"],
        "hasInference": False,
        "duplicateBlockedUntil": 0.0,
        "inferenceMs": None,
        "frames": 0,
        "potCount": 0,
        "sliderCount": 0,
        "boxNumber": initialBoxNumber,
        "potBoxCount": initialPotBoxCount,
        "sliderBoxCount": initialSliderBoxCount,
        "boxFull": initialBoxFull,
        "boxFullCategory": initialBoxFullCategory,
        "boxStopConfirmed": initialBoxStopConfirmed,
        "boxesCompleted": initialCompletedBoxes,
        "objectArmed": not initialBoxFull,
        "absentFrames": rearmMissFrames,
        "activeObjectLabel": "none",
        "lastEvent": "",
        "runnerLine": "Runner started",
        "startedAt": time.monotonic(),
    }

    if data["boxFull"]:
        data["objectArmed"] = False
        data["lastEvent"] = (
            f"{data['boxFullCategory'].upper()} CAPACITY REACHED - CONVEYOR STOPPED"
            if data["boxStopConfirmed"]
            else f"{data['boxFullCategory'].upper()} CAPACITY REACHED - FINAL ITEM IN TRANSIT"
        )

    if unsPublisher:
        unsPublisher.updateState(buildUnsState(data))

    keyReader = OperatorKeyReader()
    if keyReader.fd is None:
        print("WARNING: stdin is not an interactive terminal; new-box acknowledgement needs a TTY keypress.")

    runnerEnded = False
    lastStatusPoll = 0.0
    lastDashboard = 0.0
    lastHealthCheck = 0.0
    lastUnsHeartbeat = 0.0
    bridgeOnline = True

    def sendDetection(label):
        nonlocal bridgeOnline
        if not bridgeOnline:
            return False
        try:
            bridgeCall("setDetection", label)
            return True
        except Exception as error:
            bridgeOnline = False
            data["bridgeError"] = str(error)
            return False

    def publishState(force=False):
        nonlocal lastUnsHeartbeat
        nowValue = time.monotonic()
        if unsPublisher and (force or nowValue - lastUnsHeartbeat >= unsHeartbeatSeconds):
            unsPublisher.updateState(buildUnsState(data))
            lastUnsHeartbeat = nowValue

    def syncMcuStatus(status):
        """Keep Python's active-box state aligned with the MCU source of truth."""
        data["sortStatus"] = status
        data["inferenceReady"] = status["inferenceReady"]
        data["potBoxCount"] = status["potBoxCount"]
        data["sliderBoxCount"] = status["sliderBoxCount"]
        data["boxNumber"] = status["boxNumber"]
        data["boxFull"] = status["boxFull"]
        if status["boxFull"]:
            data["boxFullCategory"] = (
                potLabel if status["partCode"] == 2 else
                sliderLabel if status["partCode"] == 1 else "none"
            )
            data["objectArmed"] = False
            if status["state"] == 2:
                data["boxStopConfirmed"] = False
        else:
            data["boxFullCategory"] = "none"
            data["boxStopConfirmed"] = False

    def confirmBoxStop(status):
        """Publish box completion only after the MCU confirms the relay is stopped."""
        if not (
            data["boxFull"]
            and not data["boxStopConfirmed"]
            and status.get("state") == 3
            and status.get("relayStopped")
        ):
            return

        data["boxStopConfirmed"] = True
        data["boxesCompleted"] += 1
        data["lastEvent"] = f"BOX {data['boxNumber']} FULL - CONVEYOR STOPPED"

        if unsPublisher:
            unsPublisher.publishEvent(
                unsBoxTopic,
                buildUnsEvent(
                    "boxCompleted",
                    data,
                    {
                        "boxNumber": data["boxNumber"],
                        "category": data["boxFullCategory"],
                        "categoryUnits": (
                            data["potBoxCount"] if data["boxFullCategory"] == potLabel
                            else data["sliderBoxCount"]
                        ),
                        "categoryLimit": (
                            potBoxLimit if data["boxFullCategory"] == potLabel
                            else sliderBoxLimit
                        ),
                        "potUnits": data["potBoxCount"],
                        "sliderUnits": data["sliderBoxCount"],
                        "finalItemTransitDelaySeconds": boxFullStopDelaySeconds,
                        "conveyorStopped": True,
                        "operatorActionRequired": "install_new_box_then_press_N_or_ENTER",
                    },
                ),
            )

        publishState(force=True)

    def acknowledgeNewBox():
        nonlocal bridgeOnline
        nextBoxNumber = data["boxNumber"] + 1
        try:
            accepted = bool(bridgeCall("acknowledgeNewBox", str(nextBoxNumber)))
            if not accepted:
                data["runnerLine"] = "MCU rejected new-box acknowledgement"
                return

            previousBoxNumber = data["boxNumber"]
            data["boxNumber"] = nextBoxNumber
            data["potBoxCount"] = 0
            data["sliderBoxCount"] = 0
            data["boxFull"] = False
            data["boxFullCategory"] = "none"
            data["boxStopConfirmed"] = False
            data["objectArmed"] = False
            data["absentFrames"] = 0
            data["duplicateBlockedUntil"] = 0.0
            data["activeObjectLabel"] = "none"
            data["detection"] = "none"
            data["confidence"] = 0.0
            data["lastEvent"] = f"BOX {previousBoxNumber} CLOSED / BOX {nextBoxNumber} ACTIVE"

            previousStatus = data["sortStatus"] or {}
            data["sortStatus"] = {
                "state": 0 if data["inferenceReady"] else 4,
                "partCode": previousStatus.get("partCode", 0),
                "angle": previousStatus.get("angle", 0),
                "relayStopped": not data["inferenceReady"],
                "servoReady": previousStatus.get("servoReady", True),
                "detectionCode": 0,
                "boxFull": False,
                "potBoxCount": 0,
                "sliderBoxCount": 0,
                "boxNumber": nextBoxNumber,
                "inferenceReady": data["inferenceReady"],
            }
            data["runnerLine"] = "New box acknowledged; conveyor resumed"

            if unsPublisher:
                unsPublisher.publishEvent(
                    unsBoxTopic,
                    buildUnsEvent(
                        "newBoxPlaced",
                        data,
                        {
                            "previousBoxNumber": previousBoxNumber,
                            "activeBoxNumber": nextBoxNumber,
                            "operatorAcknowledged": True,
                        },
                    ),
                )
            publishState(force=True)
        except Exception as error:
            bridgeOnline = False
            data["bridgeError"] = str(error)

    try:
        while not runnerEnded:
            now = time.monotonic()

            # A key only acknowledges a full box; N or Enter means a new box is in place.
            if data["boxFull"]:
                key = keyReader.readKey()
                if key in ("n", "N", "\r", "\n"):
                    # Refresh the MCU state at keypress time so an acknowledgement
                    # just after the 3-second delay is not lost to polling latency.
                    if not data["boxStopConfirmed"] and bridgeOnline:
                        try:
                            currentStatus = parseSortStatus(
                                bridgeCall("getSortStatus")
                            )
                            if currentStatus is not None:
                                data["sortStatus"] = currentStatus
                                data["potBoxCount"] = currentStatus["potBoxCount"]
                                data["sliderBoxCount"] = currentStatus["sliderBoxCount"]
                                confirmBoxStop(currentStatus)
                        except Exception as error:
                            bridgeOnline = False
                            data["bridgeError"] = str(error)

                    if data["boxStopConfirmed"]:
                        acknowledgeNewBox()
                    else:
                        data["runnerLine"] = (
                            "New-box key ignored: waiting for final item transit delay"
                        )

            while True:
                try:
                    line = outputQueue.get_nowait()
                except queue.Empty:
                    break

                if line is None:
                    runnerEnded = True
                    break

                data["runnerLine"] = line.strip()
                inferenceMs = parseInferenceMs(line)
                if inferenceMs is not None:
                    data["inferenceMs"] = inferenceMs

                scores = parseClassification(line)
                if scores is None:
                    continue

                data["frames"] += 1
                data["scores"] = {
                    potLabel: scores.get(potLabel, 0.0),
                    sliderLabel: scores.get(sliderLabel, 0.0),
                }
                data["hasInference"] = True

                # Keep the conveyor stopped until the first valid inference arrives.
                if data["boxFull"]:
                    continue

                if not data["inferenceReady"] and bridgeOnline:
                    try:
                        started = bool(bridgeCall("startConveyor"))
                        if started:
                            data["inferenceReady"] = True
                            data["sortStatus"].update({
                                "state": 0,
                                "relayStopped": False,
                                "inferenceReady": True,
                            })
                            data["lastEvent"] = "FIRST INFERENCE / CONVEYOR STARTED"
                            if unsPublisher:
                                unsPublisher.publishEvent(
                                    unsSorterTopic,
                                    buildUnsEvent(
                                        "conveyorStartedAfterInference",
                                        data,
                                        {"firstInferenceFrame": data["frames"]},
                                    ),
                                )
                            publishState(force=True)
                        else:
                            data["runnerLine"] = "MCU rejected startConveyor request"
                    except Exception as error:
                        bridgeOnline = False
                        data["bridgeError"] = str(error)

                if not data["inferenceReady"] or not bridgeOnline:
                    continue

                potScore = data["scores"][potLabel]
                sliderScore = data["scores"][sliderLabel]
                bestScore = max(potScore, sliderScore)

                if bestScore < classificationThreshold:
                    data["absentFrames"] += 1
                    if data["absentFrames"] >= rearmMissFrames:
                        data["activeObjectLabel"] = "none"
                        data["confidence"] = 0.0
                        if time.monotonic() >= data["duplicateBlockedUntil"]:
                            data["objectArmed"] = True
                        if data["detection"] != "none":
                            data["detection"] = "none"
                            if sendDetection("none") and unsPublisher:
                                unsPublisher.publishEvent(
                                    unsDetectionTopic,
                                    buildUnsEvent("detection", data, {
                                        "label": "none",
                                        "confidence": 0.0,
                                    }),
                                )
                            publishState(force=True)
                    continue

                if (
                    not data["objectArmed"]
                    and data["absentFrames"] >= rearmMissFrames
                    and time.monotonic() >= data["duplicateBlockedUntil"]
                ):
                    data["objectArmed"] = True

                data["absentFrames"] = 0
                detectedLabel = potLabel if potScore >= sliderScore else sliderLabel
                data["confidence"] = bestScore

                previousLabel = data["activeObjectLabel"]
                if detectedLabel != previousLabel:
                    data["activeObjectLabel"] = detectedLabel

                if data["detection"] != detectedLabel:
                    data["detection"] = detectedLabel
                    sendDetection(detectedLabel)
                    if unsPublisher:
                        unsPublisher.publishEvent(
                            unsDetectionTopic,
                            buildUnsEvent("detection", data, {
                                "label": detectedLabel,
                                "confidence": bestScore,
                                "scores": data["scores"],
                            }),
                        )
                    publishState(force=True)

                if not data["objectArmed"]:
                    continue
                if time.monotonic() < data["duplicateBlockedUntil"]:
                    continue

                # Count objects independently by category within the active box.
                newPotBoxCount = data["potBoxCount"] + (1 if detectedLabel == potLabel else 0)
                newSliderBoxCount = data["sliderBoxCount"] + (1 if detectedLabel == sliderLabel else 0)
                categoryLimit = potBoxLimit if detectedLabel == potLabel else sliderBoxLimit
                categoryNewCount = newPotBoxCount if detectedLabel == potLabel else newSliderBoxCount
                isBoxFull = categoryNewCount >= categoryLimit

                registrationPayload = (
                    f"{detectedLabel},{newPotBoxCount},{newSliderBoxCount},"
                    f"{data['boxNumber']},{1 if isBoxFull else 0}"
                )

                try:
                    accepted = bool(bridgeCall("registerObject", registrationPayload))
                except Exception as error:
                    bridgeOnline = False
                    data["bridgeError"] = str(error)
                    continue

                if not accepted:
                    data["runnerLine"] = "MCU rejected item registration; retrying"
                    continue

                data["objectArmed"] = False
                data["duplicateBlockedUntil"] = time.monotonic() + duplicateSuppressionSeconds
                data["potBoxCount"] = newPotBoxCount
                data["sliderBoxCount"] = newSliderBoxCount
                data["boxFull"] = isBoxFull
                data["boxFullCategory"] = detectedLabel if isBoxFull else "none"

                if detectedLabel == potLabel:
                    data["potCount"] += 1
                    servoTargetDegree = 130
                    category = "pot"
                else:
                    data["sliderCount"] += 1
                    servoTargetDegree = 45
                    category = "slider"

                categoryUnitCount = newPotBoxCount if category == "pot" else newSliderBoxCount
                data["lastEvent"] = f"{category.upper()} / {datetime.now().strftime('%H:%M:%S')}"
                data["sortStatus"] = {
                    "state": 2 if isBoxFull else 1,
                    "partCode": 2 if category == "pot" else 1,
                    "angle": servoTargetDegree,
                    "relayStopped": False,
                    "servoReady": True,
                    "detectionCode": 1 if category == "pot" else 2,
                    "boxFull": isBoxFull,
                    "potBoxCount": newPotBoxCount,
                    "sliderBoxCount": newSliderBoxCount,
                    "boxNumber": data["boxNumber"],
                    "inferenceReady": data["inferenceReady"],
                }

                if unsPublisher:
                    unsPublisher.publishEvent(
                        unsObjectTopic,
                        buildUnsEvent(
                            "objectRegistered",
                            data,
                            {
                                "category": category,
                                "confidence": bestScore,
                                "servoTargetDegree": servoTargetDegree,
                                "boxNumber": data["boxNumber"],
                                "unitNumberInCategory": categoryUnitCount,
                                "categoryLimit": categoryLimit,
                                "potUnits": newPotBoxCount,
                                "sliderUnits": newSliderBoxCount,
                                "duplicateSuppressionSeconds": duplicateSuppressionSeconds,
                            },
                        ),
                    )

                if isBoxFull:
                    data["boxStopConfirmed"] = False
                    data["lastEvent"] = (
                        f"{category.upper()} CAPACITY REACHED - FINAL ITEM IN TRANSIT"
                    )
                    if unsPublisher:
                        unsPublisher.publishEvent(
                            unsBoxTopic,
                            buildUnsEvent(
                                "boxStopDelayStarted",
                                data,
                                {
                                    "boxNumber": data["boxNumber"],
                                    "category": category,
                                    "categoryUnits": categoryUnitCount,
                                    "categoryLimit": categoryLimit,
                                    "potUnits": newPotBoxCount,
                                    "sliderUnits": newSliderBoxCount,
                                    "finalItemTransitDelaySeconds": boxFullStopDelaySeconds,
                                    "conveyorRemainsRunning": True,
                                },
                            ),
                        )

                publishState(force=True)

            if now - lastStatusPoll >= statusPollSeconds and bridgeOnline:
                lastStatusPoll = now
                try:
                    newStatus = parseSortStatus(bridgeCall("getSortStatus"))
                    if newStatus is not None:
                        previousStatus = data["sortStatus"]
                        syncMcuStatus(newStatus)
                        confirmBoxStop(newStatus)
                        if previousStatus != newStatus:
                            publishState(force=True)
                except Exception as error:
                    bridgeOnline = False
                    data["bridgeError"] = str(error)

            if not bridgeOnline and now - lastHealthCheck >= 2.0:
                lastHealthCheck = now
                try:
                    bridgeOnline = checkBridge()
                    if bridgeOnline:
                        newStatus = parseSortStatus(bridgeCall("getSortStatus"))
                        bridgeOnline = newStatus is not None
                        if bridgeOnline:
                            syncMcuStatus(newStatus)
                            confirmBoxStop(newStatus)
                            data["bridgeError"] = ""
                            if data["hasInference"] and not data["boxFull"] and not data["inferenceReady"]:
                                started = bool(bridgeCall("startConveyor"))
                                if started:
                                    data["inferenceReady"] = True
                                    data["sortStatus"].update({
                                        "state": 0,
                                        "relayStopped": False,
                                        "inferenceReady": True,
                                    })
                            if not data["boxFull"]:
                                bridgeCall("setDetection", data["detection"])
                            publishState(force=True)
                except Exception as error:
                    bridgeOnline = False
                    data["bridgeError"] = str(error)

            data["bridgeOnline"] = bridgeOnline
            publishState()

            if now - lastDashboard >= dashboardRefreshSeconds:
                lastDashboard = now
                renderDashboard(
                    data,
                    unsPublisher.connected if unsPublisher else False,
                )

            if runnerProcess.poll() is not None and outputQueue.empty():
                runnerEnded = True
            time.sleep(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        if runnerProcess.poll() is None:
            runnerProcess.terminate()
            try:
                runnerProcess.wait(timeout=3)
            except subprocess.TimeoutExpired:
                runnerProcess.kill()

        keyReader.close()
        if unsPublisher:
            unsPublisher.updateState(buildUnsState(data))
            unsPublisher.stop()

        if sys.stdout.isatty():
            sys.stdout.write("\033[0m\033[?25h\n")
        print("Sorter stopped. The MCU retains its current box/conveyor state.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
