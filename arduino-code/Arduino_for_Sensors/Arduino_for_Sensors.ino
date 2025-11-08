#include <NewPing.h>
#define TRIGGER_PIN  7
#define ECHO_PIN1     6
#define ECHO_PIN2     5
#define ECHO_PIN3     4
#define ECHO_PIN4     3
#define ECHO_PIN5     2
#define ECHO_PIN6     8
#define ECHO_PIN7     9
#define MAX_DISTANCE 200
 
int d1, d2, d3, d4, d5, d6, d7;
int IRSensor = A0;
int IRSensor2 = A1;
int green_d = 18;
int blue_d = 8;
int cycle = 0;

String Tot_string;

NewPing sonar1(TRIGGER_PIN, ECHO_PIN1, MAX_DISTANCE);
NewPing sonar2(TRIGGER_PIN, ECHO_PIN2, MAX_DISTANCE);
NewPing sonar3(TRIGGER_PIN, ECHO_PIN3, MAX_DISTANCE);
NewPing sonar4(TRIGGER_PIN, ECHO_PIN4, MAX_DISTANCE);
NewPing sonar5(TRIGGER_PIN, ECHO_PIN5, MAX_DISTANCE);
NewPing sonar6(TRIGGER_PIN, ECHO_PIN6, MAX_DISTANCE);
NewPing sonar7(TRIGGER_PIN, ECHO_PIN7, MAX_DISTANCE);

void setup()
{
  pinMode (IRSensor, INPUT); // sensor pin INPUT
  pinMode (IRSensor2, INPUT); // sensor pin INPUT
  Serial.begin(9600); 
} 
 
void loop()
{
 int count = 0;
 for (int i = 0; i <5; i++){
   // Ignore 0 measurements
   if ( d1 != 0 || d2 != 0 || d3 != 0 || d4 != 0 || d5 != 0 || d6 != 0 || d7 !=0 ){
     delay(30);
     d1 += sonar1.ping_cm();
     delay(30);
     d2 += sonar2.ping_cm();
     delay(30);
     d5 += sonar5.ping_cm();
     delay(30);
     d3 += sonar3.ping_cm();
     delay(30);
     d4 += sonar4.ping_cm();
     delay(30);
     d6 += sonar6.ping_cm();
     delay(30);
     d7 += sonar7.ping_cm();
  
     count += 1;
   }
 
   // Check Intermidate measurements
   // Serial.println("Added measurements");
   // Serial.println(d1);
   // Serial.println(d2);
   // Serial.println(d4);   
 }
  int StatusSense = digitalRead(IRSensor);
  // float v = analogRead(IRSensor2)*0.0048828125;
  // int Bottomdist = 13*pow(v,-1);

  d1 = d1/count;
  d2 = d2/count;
  d3 = d3/count;
  d4 = d4/count;
  d5 = d5/count;
  d6 = d6/count;
  d7 = d7/count; 

  // Send to Matlab
  Tot_string = d1 + String(" ") + d2 + String(" ") + d3 + String(" ") + d4 + String(" ") + d5 + String(" ") + d6 + String(" ") + StatusSense + String(" ") + d7;
  Serial.println(Tot_string);
}
 
