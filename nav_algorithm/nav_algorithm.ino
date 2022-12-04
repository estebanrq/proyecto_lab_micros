// GX,GY,GZ,AX,AY,AZ,MX,MY,MZ,TEMP,HUM,PRES,PROX I,PROX C,PROX D,VEL_RUEDA_I,VEL_RUEDA_D,POSE_X,POSE_Y, teleoperated
//- GX,GY,GZ
//- AX,AY,AZ
//- MX,MY,MZ
//- TEMP
//- HUM
//- PRES
//- PROX 1
//- PROX 2
//- PROX 3
//- ODOM**
//- VEL_RUEDA_I
//- VEL_RUEDA_D
//- POSE_X
//- POSE_Y

//GET_distances()
//    leer utrasound1
//        send
//        delay
//        recive
//            calculo de distancia1
//    leer utrasound2
//        send
//        delay
//        recive
//            calculo de distancia2
//    leer utrasound3
//        send
//        delay
//        recive
//            calculo de distancia3
//
//
//COMPARE L AND R DISTANCES()
//    if |distR - distL| > threshold:
//        if distR > distL:
//            doblar_izquierda
//                motorA_F = 5
//                motorA_B = 0
//                motorB_F = 2
//                motorB_B = 0
//        else if distR > distL:
//            doblar derecha
//                motorA_F = 2
//                motorA_B = 0
//                motorB_F = 5
//                motorB_B = 0
//    else
//        directo
//
//READ_TH()
//    leer sensor TH

#include <Arduino_LSM9DS1.h>
#include <Arduino_HTS221.h>
#include <Arduino_LPS22HB.h>

float max_speed = 1;
float THRSHLD = 0.25;
float C_THRSHLD = 0.1;
int echoPinL = 2;  // attach pin D2 Arduino to pin Echo of HC-SR04
int trigPinL = 3;  //attach pin D3 Arduino to pin Trig of HC-SR0int
int echoPinC = 4;  // attach pin D2 Arduino to pin Echo of HC-SR04
int trigPinC = 5;  //attach pin D3 Arduino to pin Trig of HC-SR0int
int echoPinR = 6;  // attach pin D2 Arduino to pin Echo of HC-SR04
int trigPinR = 7;  //attach pin D3 Arduino to pin Trig of HC-SR0int

//motor pins:
int motor_R_F = 8;
int motor_R_B = 9;
int motor_L_F = 10;
int motor_L_B = 11;


float AX, AY, AZ = 0;
float GX, GY, GZ = 0;
float MX, MY, MZ = 0;
float TEMP = 0;    // ºC
float HUM = 0;     // %
float PRES = 0;    // kPa
float PROX_C = 0;  // cm
float PROX_L = 0;  // cm
float PROX_R = 0;  // cm
float VEL_RUEDA_I = 0;
float VEL_RUEDA_D = 0;
float POSE_X = 0;
float POSE_Y = 0;
int teleoperated = 0;

float get_distance(int _trigPin, int _echoPin);
void nav_algorithm(float max_speed, int trigPinL, int echoPinL, int trigPinC, int echoPinC, int trigPinR, int echoPinR);

void setup() {
  //setting motors:
  pinMode(motor_R_F, OUTPUT);
  pinMode(motor_R_B, OUTPUT);
  pinMode(motor_L_F, OUTPUT);
  pinMode(motor_L_B, OUTPUT);

  Serial.begin(9600);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    //while (1);
  }

  if (!HTS.begin()) {
    Serial.println("Failed to initialize humidity temperature sensor!");
    //while (1);
  }
  if (!BARO.begin()) {
    Serial.println("Failed to initialize pressure sensor!");
    //while (1);
  }
}


void loop() {
  float AX, AY, AZ;
  float GX, GY, GZ;
  float MX, MY, MZ;
  float TEMP;    // ºC
  float HUM;     // %
  float PRES;    // kPa
  float PROX_C;  // cm
  float PROX_L;  // cm
  float PROX_R;  // cm
  float VEL_RUEDA_I;
  float VEL_RUEDA_D;
  float POSE_X;
  float POSE_Y;
  int teleoperated;
  // execute navigation:
  nav_algorithm(max_speed, trigPinL, echoPinL, trigPinC, echoPinC, trigPinR, echoPinR);

  //getting Acceleration data:
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(AX, AY, AZ);
  }
  //getting Gyro data:
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(GX, GY, GZ);
  }
  //getting Magnetic data:
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(MX, MY, MZ);
  }

  //getting temperature/humidity data:
  TEMP = BARO.readTemperature();
  HUM = 10; //HTS.readHumidity();

  //getting presure data:
  PRES = BARO.readPressure();


  
  // Sending data to Bluetooth:
  Serial.print(GX);
  Serial.print(",");
  Serial.print(GY);
  Serial.print(",");
  Serial.print(GZ);
  Serial.print(",");
  Serial.print(AX);
  Serial.print(",");
  Serial.print(AY);
  Serial.print(",");
  Serial.print(AZ);
  Serial.print(",");
  Serial.print(MX);
  Serial.print(",");
  Serial.print(MY);
  Serial.print(",");
  Serial.print(MZ);
  Serial.print(",");
  Serial.print(TEMP);
  Serial.print(",");
  Serial.print(HUM);
  Serial.print(",");
  Serial.print(PRES);
  Serial.print(",");
  Serial.print(PROX_C);
  Serial.print(",");
  Serial.print(PROX_L);
  Serial.print(",");
  Serial.print(PROX_R);
  Serial.print(",");
  Serial.print(VEL_RUEDA_I);
  Serial.print(",");
  Serial.print(VEL_RUEDA_D);
  Serial.print(",");
  Serial.print(POSE_X);
  Serial.print(",");
  Serial.print(POSE_Y);
  Serial.print(",");
  Serial.println(teleoperated);
}

void nav_algorithm(float max_speed, int trigPinL, int echoPinL, int trigPinC, int echoPinC, int trigPinR, int echoPinR) {
  // getting distance data from ultrasounds:
  PROX_L = 1;    //get_distance(trigPinL, echoPinL);
  PROX_C = 0.2;  //get_distance(trigPinC, echoPinC);
  PROX_R = 3;    //get_distance(trigPinR, echoPinR);

  if (abs(PROX_L - PROX_R) > THRSHLD) {
    if (PROX_L < PROX_R) {
      drive_right(max_speed);
      delay(1);
    } else if (PROX_R < PROX_L) {
      drive_left(max_speed);
      delay(1);
    }
  } else {
    if (PROX_C < C_THRSHLD) {
      drive_backward(max_speed);
      delay(1);
    } else {
      drive_forward(max_speed);
      delay(1);
    }
  }
}

void drive_forward(int value) {
  digitalWrite(motor_R_F, value);
  digitalWrite(motor_R_B, 0);
  digitalWrite(motor_L_F, value);
  digitalWrite(motor_L_B, 0);
}
void drive_backward(int value) {
  digitalWrite(motor_R_F, 0);
  digitalWrite(motor_R_B, value);
  digitalWrite(motor_L_F, 0);
  digitalWrite(motor_L_B, value);
}

void drive_right(int value) {
  digitalWrite(motor_R_F, value / 2);
  digitalWrite(motor_R_B, 0);
  digitalWrite(motor_L_F, value);
  digitalWrite(motor_L_B, 0);
}
void drive_left(int value) {
  digitalWrite(motor_R_F, value);
  digitalWrite(motor_R_B, 0);
  digitalWrite(motor_L_F, value / 2);
  digitalWrite(motor_L_B, 0);
}

float get_distance(int _trigPin, int _echoPin) {
  long duration;  // variable for the duration of sound wave travel
  int distance;   // variable for the distance measurement
  // Clears the _trigPin condition
  digitalWrite(_trigPin, LOW);
  delayMicroseconds(2);
  // Sets the _trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(_trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(_echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;  // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  return distance;
}