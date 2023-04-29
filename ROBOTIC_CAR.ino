//FLOOR CLEANING CAR ...                                                                                                    by Mokesh Nagpal.
#include <time.h>
#include <NewPing.h>    //import libraries
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include "DHT.h"
#define sensor DHT22    //define sensor type used
const int echo_L = 2;   //initialize pin numbers                              
const int trig_L = 2;
const int echo_M = 3;
const int trig_M = 3;
const int echo_R = 4;
const int trig_R = 4;
const int echo_RR = 5;
const int trig_RR = 5;
const int echo_LL = 7;
const int trig_LL = 7;
const int L1 = 6;
const int L2 = 9;
const int R1 = 10;
const int R2 = 11;
const int button = 12;
const int pump = 13;
//A4 - data pin,A5 - clock pin
const int servo = A3;
const int tempHum = A2;


//Variables used
int rd=0;    //Reading button status 
float temp = 0.0;    //temperature and humidity
float hum = 0.0;
float sp_cm = 0.0;   //distance recorded
int sp_m=0;
float duration = 0.0;
int motor_speed = 255;    //speed of the motor can be set between 125 (minimum) and 255 (maximum)
const int max_distance = 200;   //max distance of ultrasonic sensors is set to 200cm
int distance_L = 0;    //Distance from left ultrasonic sensor
int distance_M = 0;    //Distance from middle ultrasonic sensor
int distance_R = 0;    //Distance from right ultrasonic sensor
int distance_LL = 0;   // Distance from left left ultrasonic sensor
int distance_RR = 0;   //Distance from right right ultrasonic sensor
char incomingByte = '\0';
const char msg[]="Configure your Bluetooth app !!!";
char dist[80]={'\0'};
char value[5]={'\0'};
time_t t1,t2;
int dif=0;
int c=0;
boolean m=false,r=false,l=false,rr=false,ll=false;

NewPing sonar_L(trig_L, echo_L, max_distance);    //initialize all the 3 sensors
NewPing sonar_M(trig_M, echo_M, max_distance);
NewPing sonar_R(trig_R, echo_R, max_distance);
NewPing sonar_LL(trig_LL, echo_LL, max_distance);    //initialize all the 3 sensors
NewPing sonar_RR(trig_RR, echo_RR, max_distance);
LiquidCrystal_I2C lcd(0x3F, 16, 2);    //address of lcd 
DHT dht(tempHum,sensor);
Servo ser;

void setup() 
{
  pinMode(L1, OUTPUT);    //intitialize pins as output or input
  pinMode(L2, OUTPUT);
  pinMode(R1, OUTPUT);
  pinMode(R2, OUTPUT);
  pinMode(button, INPUT);
  pinMode(pump, OUTPUT);
  digitalWrite(L1, LOW); 
  digitalWrite(L2, LOW);
  digitalWrite(R1, LOW);
  digitalWrite(R2, LOW);
  digitalWrite(pump, LOW);
  Serial.begin(9600);   //begin serial communication via bluetooth at 9600 baud rate
  dht.begin();
  lcd.init();
//  lcd.backlight();
  ser.attach(servo);
  lcd.noCursor();
  lcd.print("Welcome!");
  delay(1500);
  temperatureHumidity();
  t1=time(NULL);
  lcd.clear();
  rd=digitalRead(button);
  if(rd==HIGH)
  {
    lcd.print("Automatic Mode");
  }
  else
  {
    lcd.print("Manual Mode");
  }
  delay(500);
}
void loop() 
{
  t2=time(NULL);

  while(rd==LOW)
   {
    manualMode();  
    rd=digitalRead(button);   
    if(rd == HIGH)
    {
      moveStop();
      lcd.clear();
      lcd.print("Automatic Mode");
      delay(500);
      break;
    }
  }
  rd=digitalRead(button);
  while(rd==HIGH)
  {
    automaticMode();
    rd=digitalRead(button);
    if(rd == LOW)
    {
      moveStop();
      lcd.clear();
      lcd.print("Manual Mode");
      delay(500);
      break;
    }
  }
}

//Automatic mode
void automaticMode()
{
  dif=difftime(t2,t1);
  if(dif>120)
  {
    rotate();
  }
  if(dif>180)
    {
      temperatureHumidity();
      t1=t2;
    }
  readSensor_R();    //read distance from all the 3 sensors
  readSensor_M();
  readSensor_L();
  readSensor_LL();
  readSensor_RR();
  lcd.clear();    //print distance on LCD
  lcd.print("L=");
  lcd.print(distance_L);
  lcd.print("cm, ");
  lcd.print("M=");
  lcd.print(distance_M);
  lcd.print("cm,");
  lcd.setCursor(0, 1);
  lcd.print("R=");
  lcd.print(distance_R);
  lcd.print("cm");
  m=distance_M <= 10;    //normally 10cm is ok
  r=distance_R <= 10;
  l=distance_L <= 10;
   ll=distance_LL <= 33;   // length of car
   rr=distance_RR <= 33;   // length of car
   if(m==true&&rr==true&&ll==true)
  {
    moveStop();
    while(rr==true&&ll==true)
    {
    moveBackward();   //move back
    delay(100);
    }
    delay(1000); //time taken to cover half distance of the car19 m
    if(rr==true)
    moveRight();    
    else
    moveLeft();
    delay(10000);    //time to rotate it 180 degree
  }
  if(m==true&&r==true&&l==true)
  {
    moveStop();
    moveBackward();   //move back
    delay(2000);
    if(distance_R>distance_L)
    {
    moveRight();
    delay(2000);
    }
    else
    {
    moveLeft();
    delay(2000);
    }
  }
  else if(m==true&&l==true)
  {
    moveStop();
    moveRight();
    delay(2000);
  }
  else if(m==true&&r==true)
  {
    moveStop();
    moveLeft();
    delay(2000);
  }
  else if(r==true&&l==false)
  {
    moveStop();
    moveLeft();
    delay(2000);
  }
  else if(l==true&&r==false)
  {
    moveStop();
    moveRight();
    delay(2000);
  }
  else
  {
    moveForward();
  }
}

//Manual mode
void manualMode()
{
  if (Serial.available() > 0)   //check if any data is available
  {
    incomingByte = Serial.read();   //read incoming data
    lcd.clear();
  switch(incomingByte)    //based on received character execute respective commands
  {
    case 'F':
    moveForward();
    lcd.print("Forward");
    break;
    
    case 'B':
    moveBackward();
    lcd.print("Backward");
    break;
    
    case 'L':
    moveLeft();
    lcd.print("Left");
    break;
    
    case 'R':
    moveRight();
    lcd.print("Right");
    break;
    
    case 'S':
    moveStop();
    lcd.print("Stop");
    break;
    
    case 'P':
    digitalWrite(pump, HIGH);
    lcd.print("Pump ON");
    break;
    
    case 'p':
    digitalWrite(pump, LOW); 
    break;
    
    case '1':
    motor_speed = 155;
    break;
    
    case '2':
    motor_speed = 205;
    break;
    
    case '3':
    motor_speed = 255;
    break;

    case 'T':
    moveStop();
    rotate();
    break;

    default :
    for(int i=0;i<strlen(msg);i++)
    {
      lcd.print(msg[i]);
      if(i>15)
      {
        lcd.autoscroll();
      }
    }
    incomingByte='*';
    delay(1000);
    
  }
  incomingByte='*';
  }
}

//Rotating servo
void rotate()
{
  moveStop();
  if(c==0)
  {
  ser.write(512);
  c=1;
  }
  else
  {
  ser.write(1023);
  c=0;
  }
}

//Temperature and Humidioty
void temperatureHumidity()
{
  //no delay is given as already delay is present in the program
  hum=dht.readHumidity();
  temp=dht.readTemperature();
  sp_m=331.4 + ( 0.606 * temp ) + ( 0.0124 * hum );
  sp_cm=sp_m/100;
}

//Ultrasoonic read
void readSensor_R()    //read distance in centimeters from left sensor
{ 
  duration = sonar_R.ping_median(5);
  distance_R=(duration/2)*sp_cm;
}
void readSensor_L()    //read distance in centimeters from left sensor
{ 
  duration = sonar_L.ping_median(5);
  distance_L=(duration/2)*sp_cm;
}
void readSensor_M()    //read distance in centimeters from middle sensor
{ 
  duration = sonar_M.ping_median(5);
  distance_M=(duration/2)*sp_cm;
}

void readSensor_RR()    //read distance in centimeters from left sensor
{ 
  duration = sonar_RR.ping_median(5);
  distance_RR=(duration/2)*sp_cm;
}
void readSensor_LL()    //read distance in centimeters from left sensor
{ 
  duration = sonar_LL.ping_median(5);
  distance_LL=(duration/2)*sp_cm;
}


//Moving the car
void moveForward()
{
  digitalWrite(L1, LOW); 
  analogWrite(L2, motor_speed);
  analogWrite(R1, motor_speed);
  digitalWrite(R2, LOW);
}

void moveBackward()
{
  analogWrite(L1, motor_speed); 
  digitalWrite(L2, LOW);
  digitalWrite(R1, LOW);
  analogWrite(R2, motor_speed);
}

void moveLeft()
{
  analogWrite(L1, motor_speed); 
  digitalWrite(L2, LOW);
  analogWrite(R1, motor_speed);
  digitalWrite(R2, LOW);
}

void moveRight()
{
  digitalWrite(L1, LOW); 
  analogWrite(L2, motor_speed);
  digitalWrite(R1, LOW);
  analogWrite(R2, motor_speed);
}

void moveStop()
{
  digitalWrite(L1, LOW); 
  digitalWrite(L2, LOW);
  digitalWrite(R1, LOW);
  digitalWrite(R2, LOW);
}
