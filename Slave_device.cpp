#include <math.h>
const int EnableL = 10;
const int HighL = 8;       // LEFT SIDE MOTOR
const int LowL =9;

const int EnableR = 5;
const int HighR = 6;       //RIGHT SIDE MOTOR
const int LowR =7;
const int MEDIUM_SPEED = 100;
const int HIGH_SPEED = 150;

const int D0 = 0;   ///raspberry pi 21  LSB
const int D1 = 1;   ///raspberry pi 22
const int D2 = 2;   ///raspberry pi 23  
const int D3 = 3;   ///raspberry pi 24  MSB

int a,b,c,d,data;

void Data()
{
  a = digitalRead(D0);
  b = digitalRead(D1);
  c = digitalRead(D2);
  d = digitalRead(D3);
  data = 8*d+4*c+2*b+a; //binary to decimal conversion
}


void setup() {
Serial.begin(9600);

pinMode(EnableL, OUTPUT);
pinMode(HighL, OUTPUT);
pinMode(LowL, OUTPUT);


pinMode(EnableR, OUTPUT);
pinMode(HighR, OUTPUT);
pinMode(LowR, OUTPUT);

pinMode(D0, INPUT);
pinMode(D1, INPUT);
pinMode(D2, INPUT);
pinMode(D3, INPUT);
}


void Stop()
{
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, LOW);
  analogWrite(EnableL,0);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, LOW);
  analogWrite(EnableR,0);
  
}

void Forward()
{
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,110);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,110);
  
}


void Backward()
{
  digitalWrite(HighL, HIGH);
  digitalWrite(LowL, LOW);
  analogWrite(EnableL,10);

  digitalWrite(HighR, HIGH);
  digitalWrite(LowR, LOW);
  analogWrite(EnableR,10);
  
}

void StopDetection()
{
    analogWrite(EnableR, 0);
    analogWrite(EnableL, 0);
    delay(9000);

    analogWrite(EnableR, 150);
    analogWrite(EnableL, 150);
    delay(1000);
}
/*
void Left1()
{
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,120);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,MEDIUM_SPEED);
  
}

void Left2()
{
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,100);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,MEDIUM_SPEED);
  
}
*/
void Left3()
{
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,60);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,110);
  
}

void Left4()
{
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,0);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,100);
  
}
/*
void Right1()
{
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,MEDIUM_SPEED);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,120);
  
}

void Right2()
{
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,MEDIUM_SPEED);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,100);
  
}
*/
void Right3()
{
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,110);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,60);
  
}
void Right4()
{
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,100);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,0);
  
}


void loop() 
{
 Data();
  
 if(data == 0)
 {
    Forward();
 }
 
 else if(data == 1)
 {
    Right3();
 }
 
 else if(data == 2)
 {
    Right3();
 }
 
 else if(data == 3)
 {
    Right4();
 }
 
 else if(data == 4)
 {
    Left3();
 }
 
 else if(data == 5)
 {
    Left3();
 }
 
 else if(data == 6)
 {
    Left4();
 }

 else if(data == 7)
 {
    StopDetection();
 }
 
 else if(data > 8 )
 {
    Stop();
 }
 
}

  
