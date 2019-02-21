#include <ros.h>                        //ROS library
#include <std_msgs/Int32.h> //Documentation on http://wiki.ros.org/std_msgs
#include <Stepper.h>

ros::NodeHandle nh;             //Initialize ROS node handler
std_msgs::Int32 Distance;       //ROS requires its own data type(s)
std_msgs::Int32 Angle;
ros::Publisher chatter("chatter",&Distance);  //You need this to publish to ROS

#define POTENTIOMETER_DATA A1   //Analog in pin for wheel angle potentiometer
#define ADC_REF 3.3             //Maximum voltage on potentiometer
#define FULL_ANGLE 60.0         //Total amount of angles
#define MOTOR_VOLTAGE_PIN 7     //PWM output pin (analog out) for motor control

const byte pingPinL = 52; // Trigger Pin of Ultrasonic Sensor Left
const byte echoPinL = 53; // Echo Pin of Ultrasonic Sensor Left
const byte pingPinR = 51; // Trigger Pin of Ultrasonic Sensor Right
const byte echoPinR = 50; // Echo Pin of Ultrasonic Sensor Right

const byte LEDpinR = 49;  //Debug LEDS for sensors (not required for code to operate)
const byte LEDpinL = 48;

//Stepper motor global control variables
const byte stepperPulse = 22;
const byte stepperDir = 23;
long currentMicros = 0; long previousMicros = 0;

void setup() {

   nh.initNode();       //Intitialize ROS node
   nh.advertise(chatter); //Tell the ROS node it will be receiving information from this publisher
   Serial.begin(57600); // Starting Serial Terminal, ROS uses 57600 baud
   
   pinMode(pingPinL, OUTPUT);
   pinMode(echoPinL, INPUT);
   pinMode(pingPinR, OUTPUT);
   pinMode(echoPinR, INPUT);

   pinMode(LEDpinL, OUTPUT);
   pinMode(LEDpinR, OUTPUT);
   digitalWrite(LEDpinL, LOW);
   digitalWrite(LEDpinR, LOW);

    //FOLLOWING 3 LINES ARE FOR STEPPER MOTOR
   pinMode(pulse, OUTPUT);
   pinMode(dir, OUTPUT);
   digitalWrite(dir, LOW);

   analogWrite(MOTOR_VOLTAGE_PIN, 77);   //controls motor voltage via PWM pin and RC circuit
   //Voltage is determined by second argument of function; the maximum voltage is 5V
   //The second argument ranges from 0 to 255 and is directly proportional to 0-5V
   
}

void loop() {
   //ultrasonicPulse(200);   //Reads distance data every 200ms
   //measureAngle(0);        //Reads potentiometer angle after Ultrasonic Pulse, delay is 0 since ultrasonicPulse has 200ms delay
   stepperControl(1800, LOW);      //step command, step delay of 1800us. LOW or HIGH denotes motor direction.
}

void ultrasonicPulse(int del) {
   long duration, inches, cm;
   
   //Retrieve Data from Left Sensor
   triggerPin(pingPinL);      //Send a trigger to tell left sensor to take a measurement

   duration = pulseIn(echoPinL, HIGH);  //Measures how long the ultrasonic pulse is traveling
   
   inches = microsecondsToInches(duration);   //Unit conversions
   cm = microsecondsToCentimeters(duration);
   
   if(cm>400) {
    inches = -1;
    cm = -1;
   }
   
   if(inches<6 && inches>0) {
    digitalWrite(LEDpinL, HIGH);
   } else {
    digitalWrite(LEDpinL, LOW);
   }

   /*Serial.print("Left: ");    //uncomment to send info directly to USB port
   Serial.print(inches);
   Serial.print("in, ");
   Serial.print(cm);
   Serial.print("cm");
   Serial.println();*/

   Distance.data=cm;            //Sets the special ROS datatype to equal our cm measurement
   chatter.publish(&Distance);  //Publisher sends the data over USB to ROS node
   nh.spinOnce();               //This line is recommended after you've published information to a ROS node

   
   //Retrieve Data from Right Sensor
   triggerPin(pingPinR);     //Repeat same process as above for right sensor

   duration = pulseIn(echoPinR, HIGH);
   
   inches = microsecondsToInches(duration) + 400;
   cm = microsecondsToCentimeters(duration) + 400;

   if(cm>800) {
    inches = -2;
    cm = -2;
   }

   if(inches<406 && inches>400) {
    digitalWrite(LEDpinR, HIGH);
   } else {
    digitalWrite(LEDpinR, LOW);
   }

   Distance.data=cm;
   chatter.publish(&Distance);
   nh.spinOnce();

   delay(del);
}

void stepperControl(long del, uint8_t dir) {
  digitalWrite(stepperDir, dir);
  
  currentMicros = micros();

   if(currentMicros - previousMicros >= del){

      previousMicros = currentMicros;
      
      digitalWrite(stepperPulse,HIGH);
      
      delayMicroseconds(250); //Set Value
      
      digitalWrite(stepperPulse,LOW);
    
   }
}

float measureAngle(int del) {
  int potValue = analogRead(POTENTIOMETER_DATA); //0 - 474, -30 - 400, 30 - 569

  //float voltage = (float)potValue * ADC_REF / 569; // Integer value here depends on potentiometer
  //float degree = ((voltage * FULL_ANGLE) / ADC_REF)-30;

  float degree = ((potValue-400)*0.355)-30;

  Serial.println(potValue); //debug statement
  
  Angle.data=degree + 1050; //1050 is used to encode the message so master code can distinguish from ultrasonic data
  chatter.publish(&Angle);
  nh.spinOnce();

  delay(del);
}

void triggerPin(int pin){ //Sends a 10us pulse to the trigger pin of HC-SR04 ultrasonic sensor
   digitalWrite(pin, LOW);
   delayMicroseconds(2);
   digitalWrite(pin, HIGH);
   delayMicroseconds(10);
   digitalWrite(pin, LOW);
}

long microsecondsToInches(long microseconds) {
   return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
   return microseconds / 29 / 2;
}
