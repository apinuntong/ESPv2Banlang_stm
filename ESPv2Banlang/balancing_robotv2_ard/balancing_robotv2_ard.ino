#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
//#include <Servo.h>
#include <SPI.h>

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;


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


float P_CompCoeff = 0.95;

long data;
int x, y;

///kp = 11.36
float kp = 11.36, ki = 0.000, kd = 0.0;
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
  Wire.begin();
  delay(100);
  accelgyro.initialize();
  delay(1000);
  Serial.begin(115200);
}
float butA_motor = 0;
float butB_motor = 0;
float ftoA_motor = 0;
unsigned long period_A, period_B;



void loop()
{
  time_now = millis();

  if (time_now - time_prev >= 10)
  {
    time_prev = time_now;
    ///////////////////////////////////////////////////
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  //  ay  az gx

     //   kd = vr1 * .001f ;
    //    ki = vr2 * .01f ;
    //    kd = vr3 * .01f ;
    //    Run_Speed_T = vr4 * 0.3f ;
    //    r_angle = (-vr5 + 512) * 0.04f ;

    //    Serial.print("data : ");
    //    Serial.print(kp);         Serial.print("  ");
    //    Serial.print(ki);        Serial.print("  ");
    //    Serial.print(kd);        Serial.print("  ");
    //    Serial.print(Run_Speed_T);        Serial.print("  ");
    //    Serial.println(r_angle);

    f_angle += ((-gy / 32.8f) * (0.01f));
    float pitchAcc = atan2(ax, az) * RAD_TO_DEG;
    f_angle = P_CompCoeff * f_angle + (1.0f - P_CompCoeff) * pitchAcc;
    f_angle = (int)(f_angle * 10);
    f_angle = f_angle * 0.1f;
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
  //    if (time_now - time_prev2 >= 20)
  //  {
  //    time_prev2 = time_now;
  //    motor_a_m = (uint8_t)((((int)limit(LOutput, 1050, 1500)) - 1000) / 5);
  //    motor_b_m = (uint8_t)((((int)limit(ROutput, 1050, 1500)) - 1000) / 5);
  //    angle_m = (int8_t)f_angle;
  //    error_m = (int8_t)r_angle;
  //    kp_m = (int8_t)kp;
  //    ki_m = (int8_t)ki;
  //    kd_m = (int8_t)kd;
  //    Serial.print("&");
  //    Serial.print(",");
  //    Serial.print((int8_t)(motor_a_m)); //ความแรงเป็น%มอเตอร์ซ้าย
  //    Serial.print(",");
  //    Serial.print((int8_t)(motor_b_m)); //ความแรงเป็น%มอเตอร์ขวา
  //    Serial.print(",");
  //    Serial.print(angle_m);  // มุมจากใจโร
  //    Serial.print(",");
  //    Serial.print(error_m); //มุมที่ต้องการ
  //    Serial.print(",");
  //    Serial.print(kp_m); //  kp
  //    Serial.print(",");
  //    Serial.print(ki_m); //  ki
  //    Serial.print(",");
  //    Serial.print(kd_m); //  kd
  //    Serial.println(",");
  //  }
   vr1 = smooth(0.01f, (analogRead(VR_1) ), vr1);
  //  vr2 = smooth(0.01f, (analogRead(VR_2) ), vr2);
  //  vr3 = smooth(0.01f, (analogRead(VR_3) ), vr3);
  //  vr4 = smooth(0.01f, (analogRead(VR_4) ), vr4);
  //  vr5 = smooth(0.01f, (analogRead(VR_5) ), vr5);

}


void myPID()
{
  lastErr = error;
  error = 0.30+f_angle;
  dErr = (error - lastErr) * 10.0f;
  errSum = limit(errSum + (error * 0.01f), -100, 100);

  output_pid = (kp * error) + (ki * errSum) + (kd * dErr);

  LOutput = output_pid;
  ROutput = output_pid;
  butA_motor = limit((butA_motor + LOutput * 0.01f), -5, 5);
  //butA_motor =LOutput;
  Serial.println(f_angle);
}

void PWMControl(float set_motor1, float set_motor2)
{
  //
  if (set_motor1 > 0.0)
  {
    set_motor1 =  1000000.00f / (3200.00f * set_motor1);
    digitalWrite(dir1, 1);
    digitalWrite(dir2, 1);

  } else {

    set_motor1 = -1000000.00f / (3200.00f * set_motor1);
    digitalWrite(dir1, 0);
    digitalWrite(dir2, 0);

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
