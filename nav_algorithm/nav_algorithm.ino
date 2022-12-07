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
#include <Ultrasonic.h>

float max_speed = 200;
float THRSHLD = 25;
float C_THRSHLD = 30;
int echoPinL = 2;  // attach pin D2 Arduino to pin Echo of HC-SR04
int trigPinL = 3;  //attach pin D3 Arduino to pin Trig of HC-SR0int
int echoPinC = 4;  // attach pin D2 Arduino to pin Echo of HC-SR04
int trigPinC = 5;  //attach pin D3 Arduino to pin Trig of HC-SR0int
int echoPinR = 6;  // attach pin D2 Arduino to pin Echo of HC-SR04
int trigPinR = 7;  //attach pin D3 Arduino to pin Trig of HC-SR0int

//motor pins:
int AIA = 11;
int AIB = 10;
int BIA = 9;
int BIB = 8;


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

Ultrasonic ultrasonicL(3, 2);
Ultrasonic ultrasonicC(5, 4);
Ultrasonic ultrasonicR(7, 6);

void nav_algorithm(float max_speed);
void drive_forward(float value);
void drive_backward(float value);
void drive_right(float value);
void drive_left(float value);
float get_distance(int _trigPin, int _echoPin);

void setup() {
  //setting motors:
  pinMode(AIA, OUTPUT);
  pinMode(AIB, OUTPUT);
  pinMode(BIA, OUTPUT);
  pinMode(BIB, OUTPUT);

  analogWrite(AIA, 0);
  analogWrite(AIB, 0);
  analogWrite(BIA, 0);
  analogWrite(BIB, 0);

  Serial.begin(9600);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    //while (1);
  }

  //if (!HTS.begin()) {
  //  Serial.println("Failed to initialize humidity temperature sensor!");
  //  //while (1);
  //}
  if (!BARO.begin()) {
    Serial.println("Failed to initialize pressure sensor!");
    //while (1);
  }
}


void loop() {
  // execute navigation:
  nav_algorithm(max_speed);
  //drive_forward(max_speed);
  //delay(2000);
  //drive_backward(max_speed);
  //  delay(2000);
  //
  //drive_right(max_speed);
  //  delay(2000);
  //
  //drive_left(max_speed);
  //  delay(2000);


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
  HUM = 10;  //HTS.readHumidity(); // This measure is failing...

  //getting presure data:
  PRES = BARO.readPressure();



  // Sending data to Bluetooth:
  Serial.print(GX);
  Serial.print("\t");
  Serial.print(GY);
  Serial.print("\t");
  Serial.print(GZ);
  Serial.print("\t");
  Serial.print(AX);
  Serial.print("\t");
  Serial.print(AY);
  Serial.print("\t");
  Serial.print(AZ);
  Serial.print("\t");
  Serial.print(MX);
  Serial.print("\t");
  Serial.print(MY);
  Serial.print("\t");
  Serial.print(MZ);
  Serial.print("\t");
  Serial.print(TEMP);
  Serial.print("\t");
  Serial.print(HUM);
  Serial.print("\t");
  Serial.print(PRES);
  Serial.print("\t");
  Serial.print(PROX_C);
  Serial.print("\t");
  Serial.print(PROX_L);
  Serial.print("\t");
  Serial.print(PROX_R);
  Serial.print("\t");
  Serial.print(VEL_RUEDA_I);
  Serial.print("\t");
  Serial.print(VEL_RUEDA_D);
  Serial.print("\t");
  Serial.print(POSE_X);
  Serial.print("\t");
  Serial.print(POSE_Y);
  Serial.print("\t");
  Serial.println(teleoperated);
}

void nav_algorithm(float max_speed) {
  // getting distance data from ultrasounds:
  PROX_L = ultrasonicL.read();
  PROX_C = ultrasonicC.read();
  PROX_R = ultrasonicR.read();


  if (PROX_L < PROX_R) {
    drive_right(max_speed);
    delay(2);
  } else if (PROX_R < PROX_L) {
    drive_left(max_speed);
    delay(2);
  } else if (PROX_C < C_THRSHLD) {
    drive_backward(max_speed);
    delay(2);
  } else {
    drive_forward(max_speed);
    delay(2);
  }

  if (PROX_R < THRSHLD) {
    drive_left(max_speed);
  } else if (PROX_L < THRSHLD) {
    drive_right(max_speed);
  } else {
    drive_forward(max_speed*0.75);
  }
}

void drive_forward(float value) {
  analogWrite(AIA, value);
  analogWrite(AIB, 0);
  analogWrite(BIA, value);
  analogWrite(BIB, 0);
}
void drive_backward(float value) {
  analogWrite(AIA, 0);
  analogWrite(AIB, value);
  analogWrite(BIA, 0);
  analogWrite(BIB, value);
}

void drive_left(float value) {
  analogWrite(AIA, 0);
  analogWrite(AIB, 0);
  analogWrite(BIA, value);
  analogWrite(BIB, 0);
}
void drive_right(float value) {
  analogWrite(AIA, value);
  analogWrite(AIB, 0);
  analogWrite(BIA, 0);
  analogWrite(BIB, 0);
}
