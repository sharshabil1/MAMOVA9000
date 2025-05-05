#include <Arduino.h>
#include <ESP32Servo.h>
#include <limt_inferencing.h>

/**** CONFIGURATION ****/
// Pin Definitions
#define MIC_PIN           34  // Microphone input pin
#define MODE_BUTTON_PIN   12  // Mode selection button
#define GRIP_BUTTON_PIN   14  // Gripper control button

// Servo pins
#define SERVO_LR_PIN      25  // Left-right servo
#define SERVO_UD_PIN      26  // Up-down servo
#define SERVO_GRIP_PIN    27  // Gripper servo

// Extended servo pulse width values for increased range
#define SERVO_LR_MIN_PULSE  500   // Min pulse width for LR servo
#define SERVO_LR_MAX_PULSE  2600  // Extended max pulse width for LR servo
#define SERVO_UD_MIN_PULSE  500   // Min pulse width for UD servo
#define SERVO_UD_MAX_PULSE  2400  // Max pulse width for UD servo

// LED pins
#define LED_R_PIN         21  // RGB LED - Red
#define LED_G_PIN         22  // RGB LED - Green
#define LED_B_PIN         23  // RGB LED - Blue

// State indicator LEDs
#define STATE_LED_FREE     32  // Red LED for FREE mode
#define STATE_LED_RECORD   33  // Green LED for RECORD mode
#define STATE_LED_LOOP     13  // Blue LED for LOOP mode

// Voice recognition settings
#define CONFIDENCE_THRESHOLD 0.80  // Minimum confidence threshold for commands
#define BUFFER_SIZE EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE  // Audio buffer size

// Recording settings
#define RECORD_DURATION_MS 10000  // Maximum recording duration (10 seconds)
#define SAMPLE_RATE        16000  // Audio sample rate

// Gripper positions (servo angles)
#define GRIPPER_OPEN       10  // Angle for open gripper
#define GRIPPER_CLOSED     90  // Angle for closed gripper

// Servo movement ranges
#define LR_MIN_POS         0    // Left limit
#define LR_MAX_POS         200  // Extended right limit (beyond 180)
#define UD_MIN_POS         0    // Up limit
#define UD_MAX_POS         180  // Down limit

// Movement step sizes
#define SERVO_UD_STEP      15   // Step size for up/down movement
#define SERVO_LR_STEP      30   // Step size for left/right movement

/**** STATE DEFINITIONS ****/
enum State { FREE, RECORD, LOOP };
State currentState = FREE;

// Button state tracking
bool lastModeBtn = false;
bool lastGripBtn = false;
bool gripperOpen = false;

/**** GLOBAL VARIABLES ****/
// Audio processing
float *features = NULL;
unsigned long last_sample_time = 0;
const unsigned long SAMPLE_INTERVAL_US = 1000000 / SAMPLE_RATE;

// Servo objects
Servo servoLR, servoUD, servoGrip;

// Servo position tracking
int prevLR = 90;
int prevUD = 90;
int targetLR = 90;
int targetUD = 90;
int lastLR = 90;
int lastUD = 90;

// Motion recording
struct MotionStep {
  int lr;         // Position of LR servo
  int ud;         // Position of UD servo
  bool grip;      // Gripper state
  unsigned long time;  // Timestamp
};

MotionStep motion[200];  // Motion recording buffer
int motionIndex = 0;
bool recording = false;
unsigned long recordStart = 0;
unsigned long lastServoMoveTime = 0;

// Debug mode toggle
bool debugMode = true;

/**** SETUP FUNCTION ****/
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Configure inputs
  pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(GRIP_BUTTON_PIN, INPUT_PULLUP);

  // Configure LED outputs
  pinMode(LED_R_PIN, OUTPUT);
  pinMode(LED_G_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);
  pinMode(STATE_LED_FREE, OUTPUT);
  pinMode(STATE_LED_RECORD, OUTPUT);
  pinMode(STATE_LED_LOOP, OUTPUT);

  // Reset all LEDs
  digitalWrite(LED_R_PIN, LOW);
  digitalWrite(LED_G_PIN, LOW);
  digitalWrite(LED_B_PIN, LOW);
  
  // Set initial state LED
  digitalWrite(STATE_LED_FREE, HIGH);
  digitalWrite(STATE_LED_RECORD, LOW);
  digitalWrite(STATE_LED_LOOP, LOW);
  
  // Debug info
  Serial.print("Mode button pin: ");
  Serial.println(MODE_BUTTON_PIN);
  Serial.print("Grip button pin: ");
  Serial.println(GRIP_BUTTON_PIN);
  
  // Configure ADC
  analogReadResolution(12);

  // Initialize servo timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  
  // Configure servo PWM
  servoLR.setPeriodHertz(50);
  servoUD.setPeriodHertz(50);
  servoGrip.setPeriodHertz(50);
  
  delay(500);
  
  // Attach servos with custom pulse widths
  servoLR.attach(SERVO_LR_PIN, SERVO_LR_MIN_PULSE, SERVO_LR_MAX_PULSE);
  servoUD.attach(SERVO_UD_PIN, SERVO_UD_MIN_PULSE, SERVO_UD_MAX_PULSE);
  servoGrip.attach(SERVO_GRIP_PIN, 500, 2400);
  
  delay(200);
  
  // Initialize servo positions
  servoLR.write(90);
  delay(200);
  servoUD.write(90);
  delay(200);
  servoGrip.write(GRIPPER_CLOSED);
  delay(200);
  
  // Detach servos to prevent jitter
  detachAllServos();
  
  // Allocate memory for audio buffer
  features = (float*)malloc(BUFFER_SIZE * sizeof(float));
  if (!features) {
    Serial.println("❌ Memory allocation failed");
    while (1);
  }

  updateStateLEDs();
  Serial.println("🤖 Voice Robot Ready");
}

/**** SERVO CONTROL FUNCTIONS ****/
// Detach all servos to save power and prevent jitter
void detachAllServos() {
  servoLR.detach();
  servoUD.detach();
  servoGrip.detach();
  if (debugMode) {
    Serial.println("Servos detached");
  }
}

// Attach all servos and restore positions
void attachAllServos() {
  if (!servoLR.attached()) servoLR.attach(SERVO_LR_PIN, SERVO_LR_MIN_PULSE, SERVO_LR_MAX_PULSE);
  if (!servoUD.attached()) servoUD.attach(SERVO_UD_PIN, SERVO_UD_MIN_PULSE, SERVO_UD_MAX_PULSE);
  if (!servoGrip.attached()) servoGrip.attach(SERVO_GRIP_PIN, 500, 2400);
  
  servoLR.write(lastLR);
  delay(50);
  servoUD.write(lastUD);
  delay(50);
  servoGrip.write(gripperOpen ? GRIPPER_OPEN : GRIPPER_CLOSED);
  delay(50);
  
  if (debugMode) {
    Serial.println("Servos attached");
  }
}

// Smooth movement for left-right servo
void moveServoLR(int position) {
  position = constrain(position, LR_MIN_POS, LR_MAX_POS);
  lastLR = position;
  
  if (abs(position - prevLR) >= 2) {
    targetLR = position;
    
    if (!servoLR.attached()) {
      servoLR.attach(SERVO_LR_PIN, SERVO_LR_MIN_PULSE, SERVO_LR_MAX_PULSE);
      delay(20);
    }
    
    if (abs(prevLR - targetLR) > 5) {
      prevLR += (targetLR > prevLR) ? 5 : -5;
    } else {
      prevLR = targetLR;
    }
    
    servoLR.write(prevLR);
    lastServoMoveTime = millis();
    
    if (debugMode) {
      Serial.print("LR Servo: ");
      Serial.println(prevLR);
    }
  }
}

// Smooth movement for up-down servo
void moveServoUD(int position) {
  position = constrain(position, UD_MIN_POS, UD_MAX_POS);
  lastUD = position;
  
  if (abs(position - prevUD) >= 2) {
    targetUD = position;
    
    if (!servoUD.attached()) {
      servoUD.attach(SERVO_UD_PIN, SERVO_UD_MIN_PULSE, SERVO_UD_MAX_PULSE);
      delay(20);
    }
    
    if (abs(prevUD - targetUD) > 5) {
      prevUD += (targetUD > prevUD) ? 5 : -5;
    } else {
      prevUD = targetUD;
    }
    
    servoUD.write(prevUD);
    lastServoMoveTime = millis();
    
    if (debugMode) {
      Serial.print("UD Servo: ");
      Serial.println(prevUD);
    }
  }
}

/**** LED CONTROL FUNCTIONS ****/
// Update state indicator LEDs based on current mode
void updateStateLEDs() {
  digitalWrite(STATE_LED_FREE, LOW);
  digitalWrite(STATE_LED_RECORD, LOW);
  digitalWrite(STATE_LED_LOOP, LOW);
  
  switch (currentState) {
    case FREE:
      digitalWrite(STATE_LED_FREE, HIGH);
      Serial.println("Setting FREE LED (pin 32) HIGH");
      break;
    case RECORD:
      digitalWrite(STATE_LED_RECORD, HIGH);
      Serial.println("Setting RECORD LED (pin 33) HIGH");
      break;
    case LOOP:
      digitalWrite(STATE_LED_LOOP, HIGH);
      Serial.println("Setting LOOP LED (pin 13) HIGH");
      break;
  }
  
  delay(10);
  
  Serial.print("Current LED states - FREE (pin 32): ");
  Serial.print(digitalRead(STATE_LED_FREE) ? "ON" : "OFF");
  Serial.print(", RECORD (pin 33): ");
  Serial.print(digitalRead(STATE_LED_RECORD) ? "ON" : "OFF");
  Serial.print(", LOOP (pin 13): ");
  Serial.println(digitalRead(STATE_LED_LOOP) ? "ON" : "OFF");
}

// Set RGB LED color for feedback
void setRGB(bool r, bool g, bool b) {
  digitalWrite(LED_R_PIN, r);
  digitalWrite(LED_G_PIN, g);
  digitalWrite(LED_B_PIN, b);
}

/**** AUDIO PROCESSING ****/
// Collect audio samples from microphone
bool collectSamples() {
  unsigned long start_time = micros();
  last_sample_time = start_time;
  
  for (size_t i = 0; i < BUFFER_SIZE; i++) {
    int raw = analogRead(MIC_PIN);
    features[i] = (float)raw / 4095.0f;

    unsigned long next_sample = last_sample_time + SAMPLE_INTERVAL_US;
    while (micros() < next_sample) {
      yield();
    }
    last_sample_time = micros();
    
    if (i % 500 == 0) {
      yield();
    }
  }
  return true;
}

/**** GRIPPER CONTROL ****/
// Control the gripper servo
void setGripper(bool open) {
  gripperOpen = open;
  
  if (servoLR.attached()) servoLR.detach();
  if (servoUD.attached()) servoUD.detach();
  
  delay(50);
  
  if (!servoGrip.attached()) {
    servoGrip.attach(SERVO_GRIP_PIN, 500, 2400);
    delay(50);
  }
  
  servoGrip.write(gripperOpen ? GRIPPER_OPEN : GRIPPER_CLOSED);
  delay(200);
  
  servoGrip.detach();
  delay(50);
  
  if (!servoLR.attached() && prevLR != targetLR) servoLR.attach(SERVO_LR_PIN, SERVO_LR_MIN_PULSE, SERVO_LR_MAX_PULSE);
  if (!servoUD.attached() && prevUD != targetUD) servoUD.attach(SERVO_UD_PIN, SERVO_UD_MIN_PULSE, SERVO_UD_MAX_PULSE);
  
  lastServoMoveTime = millis();
  
  Serial.println(gripperOpen ? "🟢 Gripper Open" : "🔴 Gripper Closed");
  
  if (currentState == RECORD && recording) {
    motion[motionIndex-1].grip = gripperOpen;
  }
}

/**** COMMAND PROCESSING ****/
// Execute voice commands
void executeCommand(const char* cmd, float confidence) {
  if (debugMode) {
    Serial.print("Command: ");
    Serial.print(cmd);
    Serial.print(" (Confidence: ");
    Serial.print(confidence, 2);
    Serial.println(")");
  }
  
  attachAllServos();
  
  if (strcmp(cmd, "left") == 0) {
    lastLR = max(LR_MIN_POS, lastLR - SERVO_LR_STEP);
    moveServoLR(lastLR);
    setRGB(true, false, false);
    Serial.print("◀️ Left: ");
    Serial.println(lastLR);
  } 
  else if (strcmp(cmd, "right") == 0) {
    lastLR = min(LR_MAX_POS, lastLR + SERVO_LR_STEP);
    moveServoLR(lastLR);
    setRGB(false, true, false);
    Serial.print("▶️ Right: ");
    Serial.println(lastLR);
  } 
  else if (strcmp(cmd, "up") == 0) {
    lastUD = max(UD_MIN_POS, lastUD - SERVO_UD_STEP);
    moveServoUD(lastUD);
    setRGB(false, false, true);
    Serial.print("🔼 Up: ");
    Serial.println(lastUD);
  } 
  else if (strcmp(cmd, "down") == 0) {
    lastUD = min(UD_MAX_POS, lastUD + SERVO_UD_STEP);
    moveServoUD(lastUD);
    setRGB(true, true, false);
    Serial.print("🔽 Down: ");
    Serial.println(lastUD);
  }
  else if (strcmp(cmd, "open") == 0) {
    setGripper(true);
    setRGB(true, false, true);
  }
  else if (strcmp(cmd, "close") == 0) {
    setGripper(false);
    setRGB(false, true, true);
  }
  else {
    setRGB(false, false, false);
  }

  // Record motion if in recording mode
  if (currentState == RECORD && recording) {
    motion[motionIndex++] = {lastLR, lastUD, gripperOpen, millis() - recordStart};
    if (motionIndex >= 200) {
      recording = false;
      Serial.println("⚠️ Recording buffer full!");
      currentState = FREE;
      updateStateLEDs();
    }
  }
}

/**** INPUT HANDLING ****/
// Debounced button press detection
bool isButtonPressed(int pin, bool &lastState) {
  bool currentState = (digitalRead(pin) == LOW);
  
  static unsigned long lastDebounceTime = 0;
  const unsigned long debounceDelay = 50;
  static bool lastReading = false;
  
  if (currentState != lastReading) {
    lastDebounceTime = millis();
    lastReading = currentState;
    return false;
  }
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (currentState && !lastState) {
      lastState = true;
      Serial.println("Button press detected and debounced");
      return true;
    } else if (!currentState && lastState) {
      lastState = false;
    }
  }
  
  return false;
}

/**** MOTION PLAYBACK ****/
// Play back recorded motion sequence
void playbackMotion() {
  if (motionIndex == 0) {
    Serial.println("⚠️ No motion recorded!");
    return;
  }
  
  Serial.print("▶️ Playback: ");
  Serial.print(motionIndex);
  Serial.println(" steps");
  
  attachAllServos();
  
  unsigned long startTime = millis();
  unsigned long lastTime = 0;
  
  for (int i = 0; i < motionIndex; i++) {
    if (i > 0) {
      unsigned long targetDelay = motion[i].time - motion[i-1].time;
      delay(min(targetDelay, (unsigned long)500));
    }
    
    servoLR.write(motion[i].lr);
    servoUD.write(motion[i].ud);
    
    if (i > 0 && motion[i].grip != motion[i-1].grip) {
      servoGrip.write(motion[i].grip ? GRIPPER_OPEN : GRIPPER_CLOSED);
    }
    
    digitalWrite(STATE_LED_LOOP, i % 2 == 0 ? HIGH : LOW);
    
    yield();
  }
  
  digitalWrite(STATE_LED_LOOP, HIGH);
  Serial.println("✅ Playback complete");
  
  delay(500);
  detachAllServos();
}

/**** MAIN LOOP ****/
void loop() {
  // Auto-detach servos after inactivity
  if (millis() - lastServoMoveTime > 2000) {
    if (servoLR.attached() || servoUD.attached() || servoGrip.attached()) {
      detachAllServos();
    }
  }

  // Handle mode button
  if (isButtonPressed(MODE_BUTTON_PIN, lastModeBtn)) {
    currentState = (State)((currentState + 1) % 3);
    Serial.print("🔄 Mode switched to: ");
    Serial.println(currentState == FREE ? "Free" : currentState == RECORD ? "Record" : "Loop");

    updateStateLEDs();
    
    // Visual feedback for mode change
    for (int i = 0; i < 2; i++) {
      switch (currentState) {
        case FREE:
          digitalWrite(STATE_LED_FREE, LOW);
          delay(100);
          digitalWrite(STATE_LED_FREE, HIGH);
          break;
        case RECORD:
          digitalWrite(STATE_LED_RECORD, LOW);
          delay(100);
          digitalWrite(STATE_LED_RECORD, HIGH);
          break;
        case LOOP:
          digitalWrite(STATE_LED_LOOP, LOW);
          delay(100);
          digitalWrite(STATE_LED_LOOP, HIGH);
          break;
      }
      delay(100);
    }

    updateStateLEDs();
  }
  
  // Handle recording timeout
  if (currentState == RECORD && recording && (millis() - recordStart >= RECORD_DURATION_MS)) {
    recording = false;
    Serial.print("✅ Recording done! Steps: ");
    Serial.println(motionIndex);
    currentState = FREE;
    updateStateLEDs();
  }

  // Process audio
  if (!collectSamples()) {
    Serial.println("⚠️ Sample collection failed");
    return;
  }

  // Run Edge Impulse model for voice recognition
  signal_t signal;
  numpy::signal_from_buffer(features, BUFFER_SIZE, &signal);

  ei_impulse_result_t result;
  EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
  if (res != EI_IMPULSE_OK) {
    Serial.print("⚠️ Classifier error: ");
    Serial.println(res);
    return;
  }

  // Find best matching command
  float highestConfidence = 0;
  int bestMatchIndex = -1;
  
  for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    float confidence = result.classification[i].value;
    
    if (debugMode && confidence > 0.2) {
      Serial.print("🎤 ");
      Serial.print(result.classification[i].label);
      Serial.print(": ");
      Serial.println(confidence, 2);
    }
    
    if (confidence >= CONFIDENCE_THRESHOLD && confidence > highestConfidence) {
      highestConfidence = confidence;
      bestMatchIndex = i;
    }
  }

  // Execute command if confidence threshold met
  if (bestMatchIndex >= 0) {
    attachAllServos();
    executeCommand(result.classification[bestMatchIndex].label, highestConfidence);
  }

  delay(20);
}
