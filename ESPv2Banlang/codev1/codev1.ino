#define dir1  2
#define dir2  3
#define sck1  4
#define sck2  5
unsigned long previousMillis = 0;
unsigned long previousMillis2 = 0;

void setup() {
  Serial.begin(115200);
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(sck1, OUTPUT);
  pinMode(sck2, OUTPUT);
  digitalWrite(dir1, LOW);
  digitalWrite(dir2, LOW);

}
boolean s1=0;
boolean s2=0;
void loop() {
  int x = analogRead(A0)*2;
  x = 1023-x;
  if(x<0){
  x = -x;
  MOTOR(0,0,x,x);
  }
  if(x>0){
  MOTOR(1,1,x,x);
  }
}
void MOTOR(boolean D1,boolean D2,int SP1,int SP2){
  SP1 = 1023-SP1;
  SP2 = 1023-SP2;
   digitalWrite(dir1, D1);
  digitalWrite(dir2, D2);
    unsigned long currentMillis = micros();
  if (currentMillis - previousMillis >= SP1) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    s1 = !s1;
    digitalWrite(sck2,s1);
  }
    if (currentMillis - previousMillis2 >= SP2) {
    // save the last time you blinked the LED
    previousMillis2 = currentMillis;
    s2 = !s2;
    digitalWrite(sck2,s2);
  } 
}

