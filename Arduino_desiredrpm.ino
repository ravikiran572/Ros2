// Variables
volatile int pulseCount1 = 0;
unsigned long prevMillis = 0;
float rpm1 = 0;
float filteredRPM1 = 0;
float desiredRPM1 = 0;
float dir1 =0;
const int pulsesPerRevolution = 330;
float dt = 0.02;

// Kalman filter variables
float Q = 1, R = 1;
float x_k1 = 0, x_k2 = 0, x_k3 = 0, x_k4 = 0;
float P_k1 = 1, P_k2 = 1, P_k3 = 1, P_k4 = 1;

// PID control variables
float lx = 0.09, ly = 0.095, r = 0.04;
float kp = 10, ki = 0, kd = 0;
float prevError1 = 0;
float integral1 = 0;

// ISR for encoders (without IRAM_ATTR)
void pulseCounter1() { pulseCount1++; }

// Kalman filter function
float kalmanFilterUpdate(float z_k, float &x_k, float &P_k) {
    P_k = P_k + Q;
    float K_k = P_k / (P_k + R);
    x_k = x_k + K_k * (z_k - x_k);
    P_k = (1 - K_k) * P_k;
    return x_k;
}

// PID control function
float computePID(float desiredRPM, float currentRPM, float &prevError, float &integral) {
    float error = desiredRPM - currentRPM;
    integral += error * dt;
    integral = constrain(integral, 0, 50);
    float derivative = (error - prevError) / dt;
    prevError = error;
    return constrain(kp * error + ki * integral + kd * derivative, 0, 255);
}

void setup() {
    Serial.begin(115200);
    pinMode(4, OUTPUT); pinMode(5, OUTPUT);
    pinMode(6, OUTPUT); pinMode(7, OUTPUT);
    pinMode(9, OUTPUT); pinMode(10, OUTPUT);
    pinMode(11, OUTPUT); pinMode(12, OUTPUT);
    pinMode(30,OUTPUT); pinMode(31,OUTPUT);pinMode(32,OUTPUT);pinMode(33,OUTPUT);
    pinMode(34,OUTPUT); pinMode(35,OUTPUT);pinMode(36,OUTPUT);pinMode(37,OUTPUT);
    digitalWrite(30,LOW);
    digitalWrite(31,HIGH);
    digitalWrite(32,LOW);
    digitalWrite(33,HIGH);
    digitalWrite(34,LOW);
    digitalWrite(35,HIGH);
    digitalWrite(36,LOW);
    digitalWrite(37,HIGH);
    attachInterrupt(digitalPinToInterrupt(19), pulseCounter1, RISING);

}

void loop() {
    unsigned long currentMillis = millis();
    float timeInSeconds = currentMillis / 1000.0;
    if (Serial.available()) {
        String data = Serial.readStringUntil('\n');
        desiredRPM1 = data.toInt();  
    }
    if (currentMillis - prevMillis >= 20) {
        detachInterrupt(digitalPinToInterrupt(19));

        rpm1 = (pulseCount1 / dt) / pulsesPerRevolution * 60;
       
        filteredRPM1 = kalmanFilterUpdate(rpm1, x_k1, P_k1);
       
        pulseCount1 = 0;
        prevMillis = currentMillis;

        attachInterrupt(digitalPinToInterrupt(19), pulseCounter1, RISING);
    }

    float control1 = computePID(abs(desiredRPM1), filteredRPM1, prevError1, integral1);

    if (desiredRPM1 > 0){
      analogWrite(9, control1);
      analogWrite(10, 0);
      dir1 = 1;
    } else {
      analogWrite(9, 0);
      analogWrite(10, control1);
      dir1 = -1;
    }
    Serial.println(String(dir1*filteredRPM1)+ ',' + String(dir1*rpm1));
}
