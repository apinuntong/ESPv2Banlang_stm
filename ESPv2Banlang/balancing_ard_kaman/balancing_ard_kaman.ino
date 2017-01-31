#include <Arduino.h>
#include <Wire.h>
#include <Kalman.h>


#define RESTRICT_PITCH
Kalman kalmanX;
Kalman kalmanY;
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;



#define dir1  2
#define dir2  3
#define sck1  4
#define sck2  5
#define VR_1 0
#define VR_2 1
#define VR_3 2
#define VR_4 3
#define VR_5 6


#define Gry_offset 0.00
#define Gyr_Gain 0.00763358
#define Angle_offset 0.00f
#define offset_motor1  1275
#define offset_motor2  1275
#define pi 3.14159

void myPID(void);
void PWMControl(float set_motor1, float set_motor2);
float limit(float input, int min_limit, int max_limit);
float smooth(float alfa, float new_data, float old_data);
void MOTOR_Handle(void);
void mpu6050raed(void);

float P_CompCoeff = 0.95;

long data;
int x, y;

///kp = 11.36
float kp = 14.50, ki = 0.000, kd = 0.1;
float r_angle, f_angle, output_pid;
float Turn_Speed = 0, Turn_Speed_K = 0;
float Run_Speed = 0, Run_Speed_K = 0, Run_Speed_T = 0;
float LOutput, ROutput;

unsigned long preTime = 0;
float SampleTime = 0.08;
unsigned long lastTime;
float Input, Output;
float errSum, dErr, error, lastErr;
int timeChange;

uint8_t buf[32];
//unsigned long previousMillis = 0;        // will store last time LED was updated
unsigned long previousMillis = 0;
unsigned long previousMillis2 = 0;
// constants won't change :
const long interval = 10;
int8_t motor_a_m, motor_b_m, angle_m, error_m, error_dot_m, kp_m, ki_m, kd_m;
int Output_m;
uint32_t time_now, time_prev, time_prev2;


//Servo myservo1;
//Servo myservo2;

float vr1, vr2, vr3, vr4, vr5, vr6;

boolean s1 = 0;
boolean s2 = 0;
double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data
void setup()
{
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(sck1, OUTPUT);
  pinMode(sck2, OUTPUT);
  digitalWrite(dir1, LOW);
  digitalWrite(dir2, LOW);
  //  pinMode(VR_1, INPUT);
  //  pinMode(VR_2, INPUT);
  //  pinMode(VR_3, INPUT);
  //  pinMode(VR_4, INPUT);
  //  pinMode(VR_5, INPUT);

  Serial.begin(115200);
  Wire.begin();
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x03; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
}
float butA_motor = 0;
float butB_motor = 0;
float ftoA_motor = 0;
unsigned long period_A, period_B;



void loop()
{
  time_now = millis();
  vr1 = smooth(0.01f, (analogRead(VR_1) ), vr1);
  if (time_now - time_prev >= 5)
  {
    time_prev = time_now;
    ///////////////////////////////////////////////////

       //kp = vr1 * .1f ;
       //kd = vr2 * .01f ;
    //    kd = vr3 * .01f ;
    //    Run_Speed_T = vr4 * 0.3f ;
    //    r_angle = (-vr5 + 512) * 0.04f ;
//
//    f_angle += ((-gy / 32.8f) * (0.01f));
//    float pitchAcc = atan2(ax, az) * RAD_TO_DEG;
//    f_angle = P_CompCoeff * f_angle + (1.0f - P_CompCoeff) * pitchAcc;
//    f_angle = (int)(f_angle * 10);
//    f_angle = f_angle * 0.1f;
      mpu6050raed();
      Serial.println(kp);
      f_angle = kalAngleY;
    //Serial.print(pitchAcc); Serial.print("\t");
    //Serial.println(f_angle);

    ///////////////////////////////////////////////////

    //////////////////////////////////////////////////


    if (f_angle < 15 && f_angle > -15)
    {
      myPID();
      PWMControl( butA_motor,  butA_motor);
    } else {
      period_A = 0xffffffff;
      period_B = 0xffffffff;
      butA_motor = 0;
      errSum = 0;
    }
  }

  MOTOR_Handle();

 
    //vr2 = smooth(0.01f, (analogRead(VR_2) ), vr2);
  //  vr3 = smooth(0.01f, (analogRead(VR_3) ), vr3);
  //  vr4 = smooth(0.01f, (analogRead(VR_4) ), vr4);
  //  vr5 = smooth(0.01f, (analogRead(VR_5) ), vr5);

}


void myPID()
{
  lastErr = error;
  error = 0.20 + f_angle;
  dErr = (error - lastErr) * 10.0f;
  errSum = limit(errSum + (error * 0.01f), -100, 100);

  output_pid = (kp * error) + (ki * errSum) + (kd * dErr);

  LOutput = output_pid;
  ROutput = output_pid;
  butA_motor = limit((butA_motor + LOutput * 0.01f), -5, 5);
  //butA_motor =LOutput;
  //Serial.println(f_angle);
}

void PWMControl(float set_motor1, float set_motor2)
{
  //
  if (set_motor1 > 0.0)
  {
    set_motor1 =  1000000.00f / (3200.00f * set_motor1);
    digitalWrite(dir1, 0);
    digitalWrite(dir2, 0);

  } else {

    set_motor1 = -1000000.00f / (3200.00f * set_motor1);
    digitalWrite(dir1, 1);
    digitalWrite(dir2, 1);

  }

  period_A = set_motor1;
  period_B = period_A;

}

float limit(float input, int min_limit, int max_limit)
{
  if (input > max_limit)input = max_limit;
  if (input < min_limit)input = min_limit;
  return input;
}

float smooth(float alfa, float new_data, float old_data)
{
  return (old_data + alfa * (new_data - old_data));
}

void MOTOR_Handle(void)
{
  unsigned long currentMillis = micros();
  if (currentMillis - previousMillis >= period_A)
  {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    s1 = !s1;
    digitalWrite(sck2, s1);
  }

  if (currentMillis - previousMillis2 >= period_B)
  {
    // save the last time you blinked the LED
    previousMillis2 = currentMillis;
    s2 = !s2;
    digitalWrite(sck2, s2);
  }
}
void mpu6050raed(void) {
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];

  double dt = (double)(millis() - timer) / 1000; // Calculate delta time
  timer = millis();
  //double dt = 0.01f;
  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  /* Print Data */
  //#if 0 // Set to 1 to activate
  //  Serial.print(accX); Serial.print("\t");
  //  Serial.print(accY); Serial.print("\t");
  //  Serial.print(accZ); Serial.print("\t");
  //
  //  Serial.print(gyroX); Serial.print("\t");
  //  Serial.print(gyroY); Serial.print("\t");
  //  Serial.print(gyroZ); Serial.print("\t");
  //
  //  Serial.print("\t");
  //#endif

  //  Serial.print(roll); Serial.print("\t");
  //  Serial.print(gyroXangle); Serial.print("\t");
  //Serial.print(dt); Serial.print("\t");
  //Serial.print(kalAngleX); Serial.print("\t");

  //Serial.print("\t");
  //
  //  Serial.print(pitch); Serial.print("\t");
  //  Serial.print(gyroYangle); Serial.print("\t");
  //  Serial.print(compAngleY); Serial.print("\t");
   //Serial.print("\t");

  //#if 0 // Set to 1 to print the temperature
  //  Serial.print("\t");
  //
  //  double temperature = (double)tempRaw / 340.0 + 36.53;
  //  Serial.print(temperature); Serial.print("\t");
  //#endif

  //Serial.print("\r\n");
  //delay(10);

}

