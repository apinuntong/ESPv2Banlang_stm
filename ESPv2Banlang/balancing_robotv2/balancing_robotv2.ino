#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
//#include <Servo.h>
#include <SPI.h>

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;



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
void PWMControl(int set_motor1, int set_motor2);
float limit(float input, int min_limit, int max_limit);
float smooth(float alfa, float new_data, float old_data);

float P_CompCoeff = 0.95;

long data;
int x, y;
float kp = 10.00f, ki = 0.0, kd = 1.00;
float r_angle, f_angle, output_pid, f_angle_b;
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
unsigned long previousMillis = 0;        // will store last time LED was updated

// constants won't change :
const long interval = 10;
int8_t motor_a_m, motor_b_m, angle_m, error_m, error_dot_m, kp_m, ki_m, kd_m;
int Output_m;
uint32_t time_now, time_prev, time_prev2;


//Servo myservo1;
//Servo myservo2;

float vr1, vr2, vr3, vr4, vr5, vr6;


void setup()
{
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(4, INPUT_PULLUP);
  Wire.begin();
  Wire.setClock(400000L);
  delay(100);
  accelgyro.initialize();
  delay(1000);
  setPwmFrequency(6, 1);
  delay(1000);
  Serial.begin(115200);
}
int button1 ;
int buttonnum = 0 ;
int buttonnum2 = 0 ;
double f_angle_R=0;
void loop()
{
  time_now = millis();

  if (time_now - time_prev >= 5)
  {
    time_prev = time_now;
    ///////////////////////////////////////////////////
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  //  ay  az gx

    f_angle += ((gx / 16.4f) * (0.005f));
    double pitchAcc = atan2(-ax, az) * RAD_TO_DEG;
    f_angle = 0.95f * f_angle + 0.05f * pitchAcc;
    //f_angle_R = 0.93 * (f_angle_R + f_angle * 0.01f) + 0.07 * pitchAcc;
    f_angle_b = f_angle + 1.00;
    Serial.println(f_angle_b);
    if (f_angle_b < 1 && f_angle_b > -1&&buttonnum==1){
      buttonnum2=1;
    }
    if (f_angle_b < 15 && f_angle_b > -15&&buttonnum2==1&&digitalRead(4) == 0)
    {
      digitalWrite(3, 1);
      digitalWrite(13, 1);
      myPID();
         float a = 0.5f;
      if (f_angle_b > a || f_angle_b < -a) {
        PWMControl( LOutput,  ROutput);
      } else {
        PWMControl( 0,  0);
      }
      
      
    } else {
      buttonnum2=0;
      digitalWrite(3, 0);
      digitalWrite(13, 0);
      errSum = 0;
      PWMControl( 0,  0);
    }

    ///////////////////////////////////////////////////

    //////////////////////////////////////////////////
  }
  if (button1 == 1 && digitalRead(4) == 0) {
    buttonnum = 1;
  }
  button1 = digitalRead(4);

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
  //  vr1 = smooth(0.01f, (analogRead(VR_1) ), vr1);
  //  vr2 = smooth(0.01f, (analogRead(VR_2) ), vr2);
  //  vr3 = smooth(0.01f, (analogRead(VR_3) ), vr3);
  //  vr4 = smooth(0.01f, (analogRead(VR_4) ), vr4);
  //  vr5 = smooth(0.01f, (analogRead(VR_5) ), vr5);

}


void myPID()
{
  lastErr = error;
  error = f_angle_b;
  dErr = (error - lastErr) * 10.0f;
  errSum = limit(errSum + (error * 0.01f), -100, 100);

  output_pid = (kp * error) + (ki * errSum) + (kd * dErr);

  LOutput = output_pid;
  ROutput = output_pid;

}

void PWMControl(int set_motor1, int set_motor2)
{
  if (set_motor1 > 0) {
    digitalWrite(2, 0);
    analogWrite(6, limit(set_motor1, 0, 255));
    //Write_PWM_Pin(2, 0);
  } else if (set_motor1 < 0) {
    analogWrite(6, limit(-set_motor1, 0, 255));
    digitalWrite(2, 1);
    //Write_PWM_Pin(2, 0);
  } else {
    analogWrite(6, 0);
    digitalWrite(2, 0);
    digitalWrite(3, 0);
  }


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
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if (pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if (pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if (pin == 3 || pin == 11) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x07; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}
