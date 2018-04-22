/***************************************************
  Worcester Polytechnic Institute
  RBE501 Robot Dynamics, Team 5, Spring 2018
  Daniel Sullivan, Dominic Cupo, Yan Wang
  Haowei Zhao, Brian Wells, Xujie Dou, Lane Tobiason

  Installation Requirements:
  - You must install the rosserial library in your Arduino IDE
  - The board in the Arduino module is an unofficial Arduino Uno

  Designed to interface with four P20381RF pneumatic valves and the
  3.5mm pedal switch jack of the KLT82A.

  This code accepts ROS boolean commands as well as
  five discrete inputs from the controller box. The
  ROS and discrete inputs are OR-ed to produce digital
  outputs for five NPN transistor bases. The states of
  these transistor commands are published to the ROS NODE.

  This code expects a single byte message from ROS that
  encapsulates all commands in the least significant
  five bits as shown in the eaxmple table below.

  Similarly, the code will publish a single byte message of
  the discrete states as the examples presented in the table
  below.

  Binary       Decimal        SW   V1   V2   V3   V4
  --------------------------------------------------
  11111        31             on   on   on   on   on
  10101        21             on   off  on   off  on
  00000        0              off  off  off  off  on
  --------------------------------------------------

 ****************************************************/

// ROS Serial Setup:
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Byte.h>

ros::NodeHandle nh;           // ROS Node handle

// Individual Valve Commands
bool ROS_Sw_Cmd = 0;      // Relay (Pedal) ROS Command
bool ROS_V1_Cmd = 0;      // Valve 1 ROS Command
bool ROS_V2_Cmd = 0;      // Valve 2 ROS Command
bool ROS_V3_Cmd = 0;      // Valve 3 ROS Command
bool ROS_V4_Cmd = 0;      // Valve 4 ROS Command

// ROS Subscriber Callback Function:
void ROS_Cmd_Callback(const std_msgs::Byte& ROS_Cmd_msg) {
  // Perform bit masking to find each valve command
  ROS_Sw_Cmd = ROS_Cmd_msg.data & B10000;
  ROS_V1_Cmd = ROS_Cmd_msg.data & B01000;
  ROS_V2_Cmd = ROS_Cmd_msg.data & B00100;
  ROS_V3_Cmd = ROS_Cmd_msg.data & B00010;
  ROS_V4_Cmd = ROS_Cmd_msg.data & B00001;
}

// Instantiate ROS Subscribers
ros::Subscriber<std_msgs::Byte> ROS_Valve_Sub("ROS_Valve_Commands", &ROS_Cmd_Callback);

// Initialize ROS Publishers
std_msgs::Byte ROS_Status_msg;      // ROS Feedback Message

// Instantiate ROS Publishers
ros::Publisher ROS_Valve_Pub("ROS_Valve_Status", &ROS_Status_msg);

// Button input pins:
const int V1_Button_Pin = 2;   // Valve 1 Manual Button Read Pin
const int V2_Button_Pin = 3;   // Valve 2 Manual Button Read Pin
const int V3_Button_Pin = 4;   // Valve 3 Manual Button Read Pin
const int V4_Button_Pin = 5;   // Valve 4 Manual Button Read Pin
const int Sw_Button_Pin = 11;  // Momentary Push Button Read Pin

// Transistor base pins:
const int V1_Out_Pin = 7;   // Valve 1 Transistor Output Pin
const int V2_Out_Pin = 10;  // Valve 2 Transistor Output Pin
const int V3_Out_Pin = 9;   // Valve 3 Transistor Output Pin
const int V4_Out_Pin = 8;   // Valve 4 Transistor Output Pin
const int Sw_Out_Pin = 6;   // Relay (Pedal Switch) Transistor Output Pin

// Button states (0 == OFF, 1 == ON):
bool V1_Button_State = 0;   // Valve 1 Manual Button State
bool V2_Button_State = 0;   // Valve 2 Manual Button State
bool V3_Button_State = 0;   // Valve 3 Manual Button State
bool V4_Button_State = 0;   // Valve 4 Manual Button State
bool Sw_Button_State = 0;   // Momentary Push Button State

// Output byte msg variable:
byte Output_msg = 0;

void setup() {

  // Serial monitor initialization for debugging
  //Serial.begin(9600);

  // ROS initialization
  nh.initNode();

  // ROS subscribers
  nh.subscribe(ROS_Valve_Sub);

  // ROS advertisers
  nh.advertise(ROS_Valve_Pub);

  // Button pin inputs:
  pinMode(V1_Button_Pin, INPUT);
  pinMode(V2_Button_Pin, INPUT);
  pinMode(V3_Button_Pin, INPUT);
  pinMode(V4_Button_Pin, INPUT);
  pinMode(Sw_Button_Pin, INPUT);

  // Button pin outputs:
  pinMode(V1_Out_Pin, OUTPUT);
  pinMode(V2_Out_Pin, OUTPUT);
  pinMode(V3_Out_Pin, OUTPUT);
  pinMode(V4_Out_Pin, OUTPUT);
  pinMode(Sw_Out_Pin, OUTPUT);
}

void loop() {

  // Update button states
  V1_Button_State = digitalRead(V1_Button_Pin);
  V2_Button_State = digitalRead(V2_Button_Pin);
  V3_Button_State = digitalRead(V3_Button_Pin);
  V4_Button_State = digitalRead(V4_Button_Pin);
  Sw_Button_State = digitalRead(Sw_Button_Pin);

  // Debugging: print button states to serial monitor
  //  - note, when the ROS code is not commented out, serial output will be affected
  /*
  Serial.print("V1: ");
  Serial.print(V1_Button_State);
  Serial.print("     ");
  Serial.print("V2: ");
  Serial.print(V2_Button_State);
  Serial.print("     ");
  Serial.print("V3: ");
  Serial.print(V3_Button_State);
  Serial.print("     ");
  Serial.print("V4: ");
  Serial.print(V4_Button_State);
  Serial.print("     ");
  Serial.print("Sw: ");
  Serial.print(Sw_Button_State);
  Serial.print("     ");
  Serial.print("Output: ");
  Serial.println(Output_msg);
*/
  // Reset the output message:
  Output_msg = 0;

  // Update Relay Switch Command
  if (Sw_Button_State == HIGH || ROS_Sw_Cmd == HIGH) {
    digitalWrite(Sw_Out_Pin, HIGH);
    Output_msg = Output_msg | B10000;
  } else {
    digitalWrite(Sw_Out_Pin, LOW);
  }

  // Update Valve 1 Command
  if (V1_Button_State == HIGH || ROS_V1_Cmd == HIGH) {
    digitalWrite(V1_Out_Pin, HIGH);
    Output_msg = Output_msg | B01000;
  } else {
    digitalWrite(V1_Out_Pin, LOW);
  }

  // Update Valve 2 Command
  if (V2_Button_State == HIGH || ROS_V2_Cmd == HIGH) {
    digitalWrite(V2_Out_Pin, HIGH);
    Output_msg = Output_msg | B00100;
  } else {
    digitalWrite(V2_Out_Pin, LOW);
  }

  // Update Valve 3 Command
  if (V3_Button_State == HIGH || ROS_V3_Cmd == HIGH) {
    digitalWrite(V3_Out_Pin, HIGH);
    Output_msg = Output_msg | B00010;
  } else {
    digitalWrite(V3_Out_Pin, LOW);
  }

  // Update Valve 4 Command
  if (V4_Button_State == HIGH || ROS_V4_Cmd == HIGH) {
    digitalWrite(V4_Out_Pin, HIGH);
    Output_msg = Output_msg | B00001;
  } else {
    digitalWrite(V4_Out_Pin, LOW);
  }

  ROS_Status_msg.data = Output_msg;

  // Publish the status message
 ROS_Valve_Pub.publish( &ROS_Status_msg );

  // Update the ROS Node
  nh.spinOnce();

  // Delay for stability
  delay(1);

}
