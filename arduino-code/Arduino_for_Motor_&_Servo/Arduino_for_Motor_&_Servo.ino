#include <Servo.h>       
Servo servo_grip; 
Servo servo_lift;  

int val = 0;  //holds ascii from serial line
int enA = 11;
int enB = 6;
int in1 = 8;
int in2 = 7;
int in3 = 4;
int in4 = 2;
int in5 = 12;
int in6 = 13;
int pos = 0;

int x;
int frontDist, rightDist, backDist, leftDist;
int maxFrontDist = 7, maxBackDist= 7, maxLeftDist = 7, maxRightDist = 7;

void setup() {
  Serial.begin(9600);
  Serial.println("Rover Uno is alive!");
  // set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(in5, OUTPUT);
  pinMode(in6, OUTPUT);
  servo_grip.attach(5);
  servo_lift.attach(3);  
}

// Motor Command Functions
void MoveFwd() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(in5, HIGH);
  digitalWrite(in6, HIGH);
  analogWrite(enA, 70);
  analogWrite(enB, 70);
}

void MoveBwd() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in5, HIGH);
  digitalWrite(in6, HIGH);
  analogWrite(enA, 70);
  analogWrite(enB, 70);
}

void RotateR() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  analogWrite(enA, 70);
  analogWrite(enB, 70);
}

void RotateL() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);
  analogWrite(enA, 70);
  analogWrite(enB, 70);
}

void Stop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  digitalWrite(in5, LOW);
  digitalWrite(in6, LOW);
  analogWrite(enA, 70);
  analogWrite(enB, 70);
}

void ShiftL() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  analogWrite(enA, 70);
  analogWrite(enB, 120);
}

void ShiftR() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);
  analogWrite(enA, 70);
  analogWrite(enB, 120);
}

void loop(){
  if (Serial.available()){
    val = Serial.read();
    // Motor Command
    if (val == 'a'){
      // rotateL
      RotateL();
      delay(640);
      Stop();     
    }
    else if(val == 'd'){
      //rotateR
      RotateR();
      delay(640);
      Stop();  
    }
    else if(val == 'w'){      
      MoveFwd();
      delay(250);
      Stop();      
    }
    else if(val == 's'){
      MoveBwd();
      delay(60);
      Stop();     
    }
    else if(val == 'e'){
      RotateR();
      delay(90);
      Stop();
    }
    else if(val == 'q'){
      RotateL();
      delay(90);
      Stop();
    }
    else if(val == 'c'){
      ShiftR();
      delay(90);
      Stop();
    }
    else if(val == 'z'){
      ShiftL();
      delay(90);
      Stop();
    }
    else if(val =='f'){
      MoveFwd();
      delay(62.5);
      Stop();
    }

    // Gripper Command
    // Grab
    else if (val == 'g'){      
      delay(500);
      // open gripper
      for (int i = 45; i < 90; i+=1){
        servo_grip.write(i);
      }
      
      delay(500);
      // lower gripper      
      for (int i = 20; i < 140; i+=1){
        servo_lift.write(i);
      }
      
      delay(3000);      
      // close gripper
      for (int i = 90; i > 45; i-=1){
        servo_grip.write(i);
      }
      
      delay(1000);
      // life gripper      
      for (int i = 140; i > 20; i-=1){
        servo_lift.write(i);
      }  
    } 

    // Release
    else if (val == 'r'){      
      delay(500);
      // lower gripper      
      for (int i = 20; i < 120; i+=1){
        servo_lift.write(i);
      }
      
      delay(500);
      // open gripper
      for (int i = 45; i < 90; i+=1){
        servo_grip.write(i);
      }
      
      delay(500);
      // life gripper      
      for (int i = 120; i > 20; i-=1){
        servo_lift.write(i);
      }  
      
      delay(500);      
      // close gripper
      for (int i = 90; i > 45; i-=1){
        servo_grip.write(i);
      }
    } 
  }
  // Holding the gripper
  servo_grip.write(45);  // 45 to 90
  servo_lift.write(20); // 20 to 120
}