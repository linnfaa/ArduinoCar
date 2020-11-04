#include <Arduino.h>
#include <Servo.h>
int pinLB=11; // define pin6 as left back connect with IN1
int pinLF=6; // define pin9 as left forward connect with IN2
int pinRB=5; // define pin10 as right back connect with IN3
int pinRF=3; // define pin11 as right back connect with IN4
int inputPin = 7; // define ultrasonic receive pin (Echo)
int outputPin = 4; // define ultrasonic send pin(Trig)
int Fspeedd = 0; // forward distance
int Rspeedd = 0; // right distance
int Lspeedd = 0; // left distance
int directionn = 0; //
Servo myservo; // new myservo
int delay_time = 250; // set stable time
int Fgo = 8; // forward
int Rgo = 6; //turn right
int Lgo = 4; // turn left
int Bgo = 2;
float distanceFwd, distanceR, distanceL;

void setup()
{
  Serial.begin(9600);
  pinMode(pinLB,OUTPUT);
  pinMode(pinLF,OUTPUT);
  pinMode(pinRB,OUTPUT);
  pinMode(pinRF,OUTPUT);
  pinMode(inputPin, INPUT);
  pinMode(outputPin, OUTPUT);
  myservo.attach(9); // define the servo pin(PWM)
}
void forward() // forward
{
  analogWrite(pinRF, 235);
  analogWrite(pinRB, 0);

  analogWrite(pinLF, 250);
  analogWrite(pinLB, 0);
}
void turnL(int d) //turn left
{
  analogWrite(pinRF,235);
  analogWrite(pinRB,0);
  
  analogWrite(pinLF,0);
  analogWrite(pinLB,0);

  delay(d * 100);
}
void turnR(int e) //turn rigth
{
  analogWrite(pinRF,0);
  analogWrite(pinRB,0);
  
  analogWrite(pinLF,250);
  analogWrite(pinLB,0);

  delay(e * 100);
}
void back(int a) //back
{
  analogWrite(pinRF,0);
  analogWrite(pinRB,235);
  
  analogWrite(pinLF,0);
  analogWrite(pinLB,250);

  delay(a * 100);
}
void stop() //stop
{
  analogWrite(pinRB,0);
  analogWrite(pinRF,0);
  analogWrite(pinLB,0);
  analogWrite(pinLF,0);
}

float getDistance()
{
  float duration_us, distance_cm;
  delay(200);
  // generate 10-microsecond pulse to TRIG pin
  digitalWrite(outputPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(outputPin, LOW);
  // measure duration of pulse from ECHO pin
  duration_us = pulseIn(inputPin, HIGH);
  // calculate the distance
  distance_cm = 0.017 * duration_us;
  return distance_cm;
}

void turn(){
  int pos = 0;
  int bestPos;
  float distancePos[5];
  float max_distance = getDistance();


  // ultrasound check
  for(int i = 0; pos <= 180; pos += 45)
  {
    myservo.write(pos);
    distancePos[i] = getDistance();
    if(distancePos[i] > max_distance)
    {
      max_distance = distancePos[i];
      bestPos = pos;
    }
    i++;
    Serial.println(pos);
  }

  myservo.write(90);

  if(max_distance < 100.00)
  {
    back(15);
  }
  else if (bestPos < 90) // höger
  {
    turnR(2);
  }
  else if(bestPos > 90) // vänster
  {
    turnL(2);
  }

}


void loop()
{
  myservo.write(90); // start position
  distanceFwd = getDistance(); 
  Serial.print("\n Distance: ");
  Serial.print(distanceFwd);

  if (distanceFwd < 100.00)
  {
    stop();
    turn();
  }
  else
  {
    forward();
  }

 }