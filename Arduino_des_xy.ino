// Variables
volatile int pulseCount1 = 0; volatile int pulseCount2 = 0;
unsigned long prevMillis = 0;
float rpm1 = 0; float rpm2 = 0;
float theta_old = 0; float x_old = 0; float y_old = 0;

float filteredRPM1 = 0; float filteredRPM2 = 0;
float distance = 0; 
float dir1 =0; float dir2 =0;
const int pulsesPerRevolution = 696;
float dt = 0.02;

// Kalman filter variables
float Q = 1, R = 1;
float x_k1 = 0, x_k2 = 0, x_k3 = 0, x_k4 = 0;
float P_k1 = 1, P_k2 = 1, P_k3 = 1, P_k4 = 1;

float lx = 0.09, ly = 0.095, r = 0.02;
float kx = 0.5; float ky = 0.5;
float a = 0.07; float L = 0.175;          // length of axis


// PID control variables
float kp = 10, ki = 0, kd = 0;
float prevError1 = 0; float prevError2 = 0;
float integral1 = 0; float integral2 = 0;

// ISR for encoders (without IRAM_ATTR)
void pulseCounter1() { pulseCount1++; }
void pulseCounter2() { pulseCount2++; }

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
    pinMode(6, OUTPUT); pinMode(7, OUTPUT);
    pinMode(8, OUTPUT); pinMode(9, OUTPUT);
    pinMode(36,OUTPUT); pinMode(38,OUTPUT);
    pinMode(37,OUTPUT); pinMode(39,OUTPUT);
    digitalWrite(36,LOW);
    digitalWrite(37,HIGH);
    digitalWrite(38,LOW);
    digitalWrite(39,HIGH);
 
    attachInterrupt(digitalPinToInterrupt(19), pulseCounter1, RISING);
    attachInterrupt(digitalPinToInterrupt(20), pulseCounter1, RISING);
}

void loop() {
    unsigned long currentMillis = millis();
    float timeInSeconds = currentMillis / 1000.0;
    if (Serial.available()) {
        String data = Serial.readStringUntil('\n');
        distance = data.toInt();  
    }
    if (currentMillis - prevMillis >= 20) {
        detachInterrupt(digitalPinToInterrupt(19));

        rpm1 = (pulseCount1 / dt) / pulsesPerRevolution * 60;
        rpm2 = (pulseCount2 / dt) / pulsesPerRevolution * 60;
        filteredRPM1 = kalmanFilterUpdate(rpm1, x_k1, P_k1);
        filteredRPM2 = kalmanFilterUpdate(rpm2, x_k1, P_k1);

        float lin_vel = (2 * PI/ 60) * r * (rpm1 + rpm2) / 2;
        float ang_vel = (2 * PI / 60) * r * (rpm1 - rpm2) / L;
  
        float x_dot = lin_vel * cos(theta_old);
        float y_dot = lin_vel * sin(theta_old);
        float theta_dot = ang_vel;
  
        float x = x_old + x_dot * 0.02;
        float y = y_old + y_dot * 0.02;
        float theta = theta_old + theta_dot * 0.02;
        
        float des_x = distance;
        float des_y = 0;
  
        float des_x_dot = 0*PI*cos(2*PI*0.5*timeInSeconds);
        float des_y_dot = -0*PI*sin(2*PI*0.5*timeInSeconds);
        
        float des_linvel = cos(theta)*(des_x_dot + lx*tanh(kx*(des_x - x)/lx)) + sin(theta)*(des_y_dot + ly*tanh(ky*(des_y - y)/ly));
        float des_angvel = -(1/a)*sin(theta)*(des_x_dot + lx*tanh(kx*(des_x - x)/lx)) + (1/a)*cos(theta)*(des_y_dot + ly*tanh(ky*(des_y - y)/ly));
  
        float desiredRPM1 = (30/PI)*((1/r)*(des_linvel) + (L/(2*r))*(des_angvel));   // converting rad/sec to RPM
        float desiredRPM2 = (30/PI)*((1/r)*(des_linvel) - (L/(2*r))*(des_angvel));
  
        desiredRPM1 = constrain(desiredRPM1, 0, 240);
        desiredRPM2 = constrain(desiredRPM2, 0, 240);
        
        float control1 = computePID(abs(desiredRPM1), filteredRPM1, prevError1, integral1);
        float control2 = computePID(abs(desiredRPM2), filteredRPM2, prevError2, integral2);
    
        if (desiredRPM1 > 0){
          analogWrite(6, control1);
          analogWrite(7, 0);
          dir1 = 1;
        } else {
          analogWrite(6, 0);
          analogWrite(7, control1);
          dir1 = -1;
        }
            if (desiredRPM2 > 0){
          analogWrite(8, control2);
          analogWrite(9, 0);
          dir1 = 1;
        } else {
          analogWrite(8, 0);
          analogWrite(9, control1);
          dir1 = -1;
        }
        
        pulseCount1 = 0;
        pulseCount2 = 0;
        prevMillis = currentMillis;
        theta_old = theta;
        x_old = x;
        y_old = y;

        attachInterrupt(digitalPinToInterrupt(19), pulseCounter1, RISING);
        attachInterrupt(digitalPinToInterrupt(20), pulseCounter2, RISING);
    }
}
