// Constants for calibration and motion detection
const int CALIBRATION_TIME_SEC = 30;  // Sensor calibration time in seconds (10-60 secs per datasheet)
const unsigned long MOTION_PAUSE_MS = 5000;  // Time sensor needs to be LOW to assume no motion

// Pin definitions
const int PIR_PIN = 3;    // Digital pin connected to PIR sensor's output
const int LED_PIN = 13;   // Pin for LED

// Variables for timing and state
unsigned long motionLowStartTime = 0;  // Time when sensor output went LOW
bool isMotionDetected = false;         // State tracking if motion is detected
bool recordLowStartTime = false;       // Flag to record the time when sensor output goes LOW

/////////////////////////////
// SETUP
void setup() {
  Serial.begin(9600);
  pinMode(PIR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(PIR_PIN, LOW);

  // Give sensor time to calibrate
  Serial.print("Calibrating sensor ");
  for (int i = 0; i < CALIBRATION_TIME_SEC; i++) {
    Serial.print(".");
    delay(1000);  // Delay for 1 second
  }
  Serial.println(" done");
  Serial.println("SENSOR ACTIVE");
  delay(50);  // Short delay before starting
}

/////////////////////////////
// LOOP
void loop() {
  int pirState = digitalRead(PIR_PIN);  // Read the PIR sensor's state

  if (pirState == HIGH) {
    // Motion detected
    digitalWrite(LED_PIN, HIGH);  // Turn on the LED

    if (!isMotionDetected) {
      // Transition from no motion to motion
      isMotionDetected = true;
      Serial.println("---");
      Serial.print("Motion detected at ");
      Serial.print(millis() / 1000);  // Print time in seconds
      Serial.println(" sec");
      delay(50);
    }

    // Set flag to record time when motion stops
    recordLowStartTime = true;
  } else {
    // No motion detected
    digitalWrite(LED_PIN, LOW);  // Turn off the LED

    if (recordLowStartTime) {
      // Record the time when the PIR sensor goes LOW
      motionLowStartTime = millis();
      recordLowStartTime = false;
    }

    // Check if the sensor has been LOW for the specified pause duration
    if (isMotionDetected && millis() - motionLowStartTime > MOTION_PAUSE_MS) {
      isMotionDetected = false;  // Reset motion detection state
      Serial.print("Motion ended at ");
      Serial.print((millis() - MOTION_PAUSE_MS) / 1000);  // Print time in seconds
      Serial.println(" sec");
      delay(50);
    }
  }
}
