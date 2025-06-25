// Variables
volatile int pulseCount = 0;      // Counter for the number of pulses
unsigned long prevMillis = 0;     // To keep track of time
float rpm = 0;                    // Motor speed in RPM
int pulsesPerRevolution = 330;     // Encoder pulses per revolution (adjust according to your motor)


// PID Variables
float desiredRPM = 100;            // Setpoint for motor 1 RPM (change as needed)

float kp = 3;                    // Proportional gain
float ki = 8;                    // Integral gain
float kd = 0;                    // Derivative gain

float prevError = 0;
float integral = 0;
float dt = 0.02;                    // Time step (seconds) -> 100ms

// Kalman filter variables for motor 1
float x_k = 0;                     // Estimated RPM (Kalman)
float P_k = 1;                     // Error covariance
float Q = 1;                    // Process noise covariance
float R = 1;                       // Measurement noise covariance
float K_k = 0;                     // Kalman gain



// Interrupt Service Routine (ISR) for the encoder
void pulseCounter() {
  pulseCount++;
}


// PID Control Function for Motor 1
float computePID(float desiredRPM, float currentRPM, float &prevError, float &integral) {
  float error = desiredRPM - currentRPM;
  integral += error * dt;
  float derivative = (error - prevError) / dt;
  prevError = error;

  // PID formula
  float output = kp * error + ki * integral + kd * derivative;

  // Limit the output to a range suitable for motor control (e.g., 0-255 for PWM)
  output = constrain(output, 0, 1023);

  return output;
}



// Kalman filter update function for motor 1
float kalmanFilterUpdate(float z_k, float &x_k, float &P_k, float Q, float R) {
  // Prediction step
  P_k = P_k + Q;

  // Update step
  K_k = P_k / (P_k + R);
  x_k = x_k + K_k * (z_k - x_k);
  P_k = (1 - K_k) * P_k;

  return x_k;
}



void setup() {
  Serial.begin(115200);                       // Initialize serial communication (ESP8266 usually runs better at 115200 baud)
  attachInterrupt(digitalPinToInterrupt(19), pulseCounter, RISING);
  for (int i = 4; i <= 12; i++) {
    pinMode(i, OUTPUT);
  }
  for (int i = 30; i < 38; i++) {
    pinMode(i, OUTPUT);
  }

  digitalWrite(30, LOW);
  digitalWrite(32, LOW);
  digitalWrite(34, LOW);
  digitalWrite(36, LOW);
  digitalWrite(31, HIGH);
  digitalWrite(33, HIGH);
  digitalWrite(35, HIGH);
  digitalWrite(37, HIGH);


}

void loop() {

  unsigned long currentMillis = millis();    // Get current time

  if (currentMillis - prevMillis >= 20) {  // Every 100 ms
    detachInterrupt(digitalPinToInterrupt(19)); // Disable interrupt temporarily

    // Calculate RPM
    rpm = (pulseCount / 0.02) / ((float)pulsesPerRevolution) * 60.0;  // Convert pulses to RPM

    // Apply Kalman filter to RPM values
    float filteredRPM = kalmanFilterUpdate(rpm, x_k, P_k, Q, R);        // Filtered RPM for motor

    // PID control to adjust motor speeds
    float controlOutput = computePID(desiredRPM, rpm, prevError, integral);

    // Here you would typically set the motor speed based on the control output (e.g., via PWM)
    analogWrite(11, controlOutput);  // Motor 1 control

    // Print the result
    Serial.print("RPM: ");
    Serial.println(filteredRPM);

    // Reset pulse count and timer
    pulseCount = 0;
    prevMillis = currentMillis;

    // Re-enable interrupts
    attachInterrupt(digitalPinToInterrupt(19), pulseCounter, RISING);
  }

}
