/*
  CONVEYOR LINE OBJECT DETECTION AND SORTER
  Edge Impulse + Arduino UNO Q
  Roni Bandini - 11/2026

*/

#include <Servo.h>
#include <Arduino_RouterBridge.h>

// -----------------------------------------------------------------------------
// Hardware configuration
// -----------------------------------------------------------------------------

const int SERVO_PIN = 5;
const int RELAY_PIN = 7;

const int SLIDER_DEGREE = 45;
const int POT_DEGREE = 130;
const int SERVO_START_DEGREE = 90;

// LOW = conveyor running; HIGH = conveyor stopped.
const bool RELAY_ACTIVE_LOW = false;

// Reset button
const int BOX_RESET_BUTTON_PIN = 6;
const unsigned long BUTTON_DEBOUNCE_MS = 40;

const unsigned long RED_FLASH_INTERVAL_MS = 350;
const unsigned long BOX_FULL_STOP_DELAY_MS = 3000;
const unsigned long SERVO_RETURN_DELAY_MS = 5000;

Servo sorterServo;

// UNO Q MCU-controlled RGB LED 3 uses active-low outputs.
const int LED_RED_PIN = LED3_R;
const int LED_GREEN_PIN = LED3_G;
const int LED_BLUE_PIN = LED3_B;

enum SortState : uint8_t {
  SORT_IDLE = 0,
  SORT_OBJECT_REGISTERED = 1,
  SORT_BOX_STOP_PENDING = 2,
  SORT_BOX_FULL = 3,
  SORT_WAITING_FOR_INFERENCE = 4
};

enum PartCode : uint8_t {
  PART_NONE = 0,
  PART_SLIDER = 1,
  PART_POT = 2
};

enum DetectionCode : uint8_t {
  DETECTION_NONE = 0,
  DETECTION_POT = 1,
  DETECTION_SLIDER = 2
};

SortState sortState = SORT_IDLE;
PartCode lastPartCode = PART_NONE;
DetectionCode detectionCode = DETECTION_NONE;

bool servoReady = false;
bool relayStopped = false;
bool boxFull = false;
bool boxStopPending = false;
bool inferenceReady = false;
bool flashOn = false;

int currentServoAngle = SERVO_START_DEGREE;
int potBoxCount = 0;
int sliderBoxCount = 0;
int boxNumber = 1;

unsigned long lastFlashAt = 0;
unsigned long boxFullRegisteredAt = 0;
unsigned long servoCommandAt = 0;
bool servoReturnPending = false;

int lastButtonReading = HIGH;
int stableButtonState = HIGH;
unsigned long buttonLastChangeAt = 0;

// -----------------------------------------------------------------------------
// RGB LED 3
// -----------------------------------------------------------------------------

void writeRgb(bool redOn, bool greenOn, bool blueOn) {
  digitalWrite(LED_RED_PIN, redOn ? LOW : HIGH);
  digitalWrite(LED_GREEN_PIN, greenOn ? LOW : HIGH);
  digitalWrite(LED_BLUE_PIN, blueOn ? LOW : HIGH);
}

void applyDetectionLed(DetectionCode code) {
  detectionCode = code;
  if (boxFull) return;

  if (code == DETECTION_POT) {
    writeRgb(true, true, false);       // Yellow: pot
  } else if (code == DETECTION_SLIDER) {
    writeRgb(true, false, false);      // Red: slider
  } else {
    writeRgb(false, true, false);      // Green: no detection
  }
}

void updateBoxFullLed(unsigned long now) {
  // Red flashing begins only after the final-item transit window ends.
  if (!boxFull || boxStopPending) return;

  if (now - lastFlashAt >= RED_FLASH_INTERVAL_MS) {
    lastFlashAt = now;
    flashOn = !flashOn;
    writeRgb(flashOn, false, false);
  }
}

// -----------------------------------------------------------------------------
// Conveyor relay
// -----------------------------------------------------------------------------

void setConveyorRunning(bool running) {
  const int stopLevel = RELAY_ACTIVE_LOW ? LOW : HIGH;
  const int runLevel = RELAY_ACTIVE_LOW ? HIGH : LOW;
  digitalWrite(RELAY_PIN, running ? runLevel : stopLevel);
  relayStopped = !running;
}

// Schedule the servo return without blocking the MCU loop or Bridge RPC handler.
void scheduleServoReturn() {
  servoCommandAt = millis();
  servoReturnPending = true;
}

void updateServoReturn(unsigned long now) {
  if (!servoReturnPending) return;
  if (now - servoCommandAt < SERVO_RETURN_DELAY_MS) return;

  sorterServo.write(SERVO_START_DEGREE);
  currentServoAngle = SERVO_START_DEGREE;
  servoReturnPending = false;
  Serial.println("Servo returned to middle position (90 degrees).");
}

// -----------------------------------------------------------------------------
// Parse Python payload: potCount,sliderCount,boxNumber,boxFull
// -----------------------------------------------------------------------------

bool parseBoxPayload(
  String payload,
  int &newPotCount,
  int &newSliderCount,
  int &newBoxNumber,
  bool &newBoxFull
) {
  payload.trim();

  int comma1 = payload.indexOf(',');
  if (comma1 < 0) return false;
  int comma2 = payload.indexOf(',', comma1 + 1);
  if (comma2 < 0) return false;
  int comma3 = payload.indexOf(',', comma2 + 1);
  if (comma3 < 0) return false;

  String potText = payload.substring(0, comma1);
  String sliderText = payload.substring(comma1 + 1, comma2);
  String boxText = payload.substring(comma2 + 1, comma3);
  String fullText = payload.substring(comma3 + 1);

  potText.trim();
  sliderText.trim();
  boxText.trim();
  fullText.trim();

  if (!potText.length() || !sliderText.length() || !boxText.length()) return false;
  if (fullText != "0" && fullText != "1") return false;

  newPotCount = potText.toInt();
  newSliderCount = sliderText.toInt();
  newBoxNumber = boxText.toInt();
  newBoxFull = (fullText == "1");

  return newPotCount >= 0 && newSliderCount >= 0 && newBoxNumber >= 1;
}

// -----------------------------------------------------------------------------
// RPC handlers
// -----------------------------------------------------------------------------

bool waitForInference() {
  // Do not interrupt a final-item transit window if Python restarts mid-cycle.
  if (boxFull) {
    Serial.println(boxStopPending
      ? "Box capacity reached: final item is still travelling."
      : "Box capacity reached: conveyor stopped; waiting for a new box.");
    return true;
  }

  inferenceReady = false;
  setConveyorRunning(false);
  sortState = SORT_WAITING_FOR_INFERENCE;
  applyDetectionLed(DETECTION_NONE);
  Serial.println("Waiting for first valid Edge Impulse inference.");
  return true;
}

bool startConveyor() {
  if (!servoReady || boxFull) return false;
  inferenceReady = true;
  setConveyorRunning(true);
  sortState = SORT_IDLE;
  Serial.println("First inference received. Conveyor running.");
  return true;
}

bool setDetection(String label) {
  label.trim();
  label.toLowerCase();

  if (label == "pot") {
    applyDetectionLed(DETECTION_POT);
  } else if (label == "slider") {
    applyDetectionLed(DETECTION_SLIDER);
  } else if (label == "none") {
    applyDetectionLed(DETECTION_NONE);
  } else {
    return false;
  }

  return true;
}

bool registerObject(String registrationPayload) {
  registrationPayload.trim();
  int separator = registrationPayload.indexOf(',');
  if (separator < 1) return false;

  String label = registrationPayload.substring(0, separator);
  String boxPayload = registrationPayload.substring(separator + 1);
  label.trim();
  label.toLowerCase();

  if (!servoReady || boxFull || !inferenceReady) return false;

  int newPotCount = 0;
  int newSliderCount = 0;
  int newBoxNumber = 0;
  bool newBoxFull = false;

  if (!parseBoxPayload(
    boxPayload,
    newPotCount,
    newSliderCount,
    newBoxNumber,
    newBoxFull
  )) {
    return false;
  }

  if (newBoxNumber != boxNumber) return false;

  // Exactly one category counter must increase by one; the other must stay unchanged.
  if (label == "pot") {
    if (newPotCount != potBoxCount + 1 || newSliderCount != sliderBoxCount) {
      return false;
    }
    lastPartCode = PART_POT;
    currentServoAngle = POT_DEGREE;
    applyDetectionLed(DETECTION_POT);

  } else if (label == "slider") {
    if (newSliderCount != sliderBoxCount + 1 || newPotCount != potBoxCount) {
      return false;
    }
    lastPartCode = PART_SLIDER;
    currentServoAngle = SLIDER_DEGREE;
    applyDetectionLed(DETECTION_SLIDER);

  } else {
    return false;
  }

  sorterServo.write(currentServoAngle);
  potBoxCount = newPotCount;
  sliderBoxCount = newSliderCount;

  if (newBoxFull) {
    // Lock out further registrations now, but let the last item travel
    // for three seconds before stopping the conveyor.
    boxFull = true;
    boxStopPending = true;
    boxFullRegisteredAt = millis();
    sortState = SORT_BOX_STOP_PENDING;
    setConveyorRunning(true);
    Serial.println("Category capacity reached; final-item transit window: 3000 ms.");
  } else {
    sortState = SORT_OBJECT_REGISTERED;
    setConveyorRunning(true);
  }

  Serial.print("Object registered: ");
  Serial.print(label);
  Serial.print(" / servo: ");
  Serial.print(currentServoAngle);
  Serial.print(" / box: ");
  Serial.print(boxNumber);
  Serial.print(" / POT: ");
  Serial.print(potBoxCount);
  Serial.print(" / SLIDER: ");
  Serial.println(sliderBoxCount);

  scheduleServoReturn();
  return true;
}

bool resetCurrentBox(int nextBoxNumber, const char *acknowledgementSource) {
  // Both the terminal N key and the physical D6 button use this same interlock.
  // A reset is accepted only after the final-item transit delay has elapsed
  // and the MCU confirms that the conveyor relay is stopped.
  if (!boxFull || boxStopPending || sortState != SORT_BOX_FULL || !relayStopped) {
    return false;
  }

  if (nextBoxNumber != boxNumber + 1) return false;

  boxNumber = nextBoxNumber;
  potBoxCount = 0;
  sliderBoxCount = 0;
  boxFull = false;
  boxStopPending = false;
  flashOn = false;
  boxFullRegisteredAt = 0;
  sortState = inferenceReady ? SORT_IDLE : SORT_WAITING_FOR_INFERENCE;

  setConveyorRunning(inferenceReady);
  scheduleServoReturn();
  applyDetectionLed(DETECTION_NONE);

  Serial.print("New box acknowledged via ");
  Serial.print(acknowledgementSource);
  Serial.print(". Active box: ");
  Serial.println(boxNumber);
  Serial.println("POT and SLIDER category counters reset.");
  Serial.println(inferenceReady ? "Conveyor running." : "Waiting for first inference.");
  return true;
}

bool acknowledgeNewBox(String nextBoxNumberText) {
  nextBoxNumberText.trim();
  if (!nextBoxNumberText.length()) return false;

  int nextBoxNumber = nextBoxNumberText.toInt();
  return resetCurrentBox(nextBoxNumber, "terminal N key");
}

void updateBoxResetButton(unsigned long now) {
  int reading = digitalRead(BOX_RESET_BUTTON_PIN);

  if (reading != lastButtonReading) {
    lastButtonReading = reading;
    buttonLastChangeAt = now;
  }

  if (now - buttonLastChangeAt < BUTTON_DEBOUNCE_MS) return;
  if (reading == stableButtonState) return;

  stableButtonState = reading;

  // Trigger once on the debounced HIGH -> LOW press edge.
  if (stableButtonState == LOW) {
    if (boxFull && !boxStopPending && sortState == SORT_BOX_FULL && relayStopped) {
      resetCurrentBox(boxNumber + 1, "D6 push button");
    } else {
      Serial.println("D6 button pressed; ignored because no stopped full box is waiting.");
    }
  }
}

String getSortStatus() {
  // CSV (11 fields): state,partCode,angle,relayStopped,servoReady,
  // detectionCode,boxFull,potBoxCount,sliderBoxCount,boxNumber,inferenceReady
  return String((int)sortState) + "," +
         String((int)lastPartCode) + "," +
         String(currentServoAngle) + "," +
         String(relayStopped ? 1 : 0) + "," +
         String(servoReady ? 1 : 0) + "," +
         String((int)detectionCode) + "," +
         String(boxFull ? 1 : 0) + "," +
         String(potBoxCount) + "," +
         String(sliderBoxCount) + "," +
         String(boxNumber) + "," +
         String(inferenceReady ? 1 : 0);
}

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);
  writeRgb(false, true, false);

  pinMode(RELAY_PIN, OUTPUT);

  pinMode(BOX_RESET_BUTTON_PIN, INPUT_PULLUP);
  lastButtonReading = digitalRead(BOX_RESET_BUTTON_PIN);
  stableButtonState = lastButtonReading;
  buttonLastChangeAt = millis();

  inferenceReady = false;
  boxFull = false;
  boxStopPending = false;
  sortState = SORT_WAITING_FOR_INFERENCE;
  setConveyorRunning(false);

  sorterServo.attach(SERVO_PIN);
  servoReady = sorterServo.attached();
  if (servoReady) {
    sorterServo.write(SERVO_START_DEGREE);
    currentServoAngle = SERVO_START_DEGREE;
  }

  Serial.println("=== CONVEYOR LINE OBJECT DETECTION AND SORTER ===");
  Serial.println("Edge Impulse + Arduino UNO Q");
  Serial.println("Roni Bandini // 11/2026");
  Serial.print("Servo ready: ");
  Serial.println(servoReady ? "YES" : "NO");
  Serial.println("Servo targets: slider=45, pot=130, middle=90");
  Serial.println("POT and SLIDER box limits are configured independently in Python.");
  Serial.println("Box reset button: D6 to GND (INPUT_PULLUP).");
  Serial.println("New boxes can be acknowledged with terminal N/ENTER or the D6 button.");
  Serial.println("Conveyor default: STOPPED - waiting for first inference");

  if (!Bridge.begin()) {
    Serial.println("FATAL: Bridge.begin() failed");
    return;
  }

  Bridge.provide_safe("waitForInference", waitForInference);
  Bridge.provide_safe("startConveyor", startConveyor);
  Bridge.provide_safe("registerObject", registerObject);
  Bridge.provide_safe("setDetection", setDetection);
  Bridge.provide_safe("acknowledgeNewBox", acknowledgeNewBox);
  Bridge.provide_safe("getSortStatus", getSortStatus);

  Serial.println("Bridge ready.");
  Serial.println("RPC methods registered: waitForInference, startConveyor, registerObject, setDetection, acknowledgeNewBox, getSortStatus");
  Serial.println("Full-box final-item transit delay: 3000 ms");
  Serial.println("Servo returns to middle 5 seconds after detection or box reset (non-blocking).");
}

// -----------------------------------------------------------------------------
// Main loop: service servo return, box transit delay and flashing red LED.
// -----------------------------------------------------------------------------

void loop() {
  unsigned long now = millis();

  updateServoReturn(now);
  updateBoxResetButton(now);

  if (boxStopPending && now - boxFullRegisteredAt >= BOX_FULL_STOP_DELAY_MS) {
    boxStopPending = false;
    setConveyorRunning(false);
    sortState = SORT_BOX_FULL;

    flashOn = true;
    lastFlashAt = now;
    writeRgb(true, false, false);

    Serial.print("Box ");
    Serial.print(boxNumber);
    Serial.println(" conveyor stopped after final-item transit window.");
  }

  updateBoxFullLed(now);
}
