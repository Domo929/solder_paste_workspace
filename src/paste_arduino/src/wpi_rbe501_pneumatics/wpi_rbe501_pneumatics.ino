/***************************************************
  Worcester Polytechnic Institute
  RBE501 Robot Dynamics, Team 5, Spring 2018
  Daniel Sullivan, Dominic Cupo, Yan Wang
  Haowei Zhao, Brian Wells, Xujie Dou, Lane Tobiason

  Installation Requirements:
  - You must install the rosserial library in your Arduino IDE
  - You must install the Sparkfun AVR boards
      - see:https://github.com/sparkfun/Arduino_Boards
  - The board this code is running in is the
      - Sparkfun Pro Micro (CHOOSE THE 5V OPTION under "Processor"

  Designed to interface with four P20381RF pneumatic valves

  This code accepts ROS boolean commands as well as
  four discrete inputs from the controller box. The
  ROS and discrete inputs are OR-ed to produce digital
  outputs for four NPN transistor bases. The states of
  these transistor commands are published to the ROS NODE.

  This code expects a single byte message from ROS that
  encapsulates all valve commands in the least significant
  four bits as shown in the table below.

  Similarly, the code will publish a single byte message of
  the discrete states as presented in the table below.

  Binary       Decimal        V1   V2   V3   V4
  -----------------------------------------------
  1111         15             on   on   on   on
  1110         14             on   on   on   off
  1101         13             on   on   off  on
  1100         12             on   on   off  off
  1011         11             on   off  on   on
  1010         10             on   off  on   off
  1001         9              on   off  off  on
  1000         8              on   off  off  off
  0111         7              off  on   on   on
  0110         6              off  on   on   off
  0101         5              off  on   off  on
  0100         4              off  on   off  off
  0011         3              off  off  on   on
  0010         2              off  off  on   off
  0001         1              off  off  off  on
  0000         0              off  off  off  off

 ****************************************************/

// ROS Serial Setup:
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Byte.h>

ros::NodeHandle nh;           // ROS Node handle

// Individual Valve Commands
bool ROS_V1_Cmd = 0;      // Valve 1 ROS Command
bool ROS_V2_Cmd = 0;      // Valve 2 ROS Command
bool ROS_V3_Cmd = 0;      // Valve 3 ROS Command
bool ROS_V4_Cmd = 0;      // Valve 4 ROS Command

// ROS Subscriber Callback Function:
void ROS_Cmd_Callback(const std_msgs::Byte& ROS_Cmd_msg) {
  // Perform bit masking to find each valve command
  ROS_V1_Cmd = ROS_Cmd_msg.data & B1000;
  ROS_V2_Cmd = ROS_Cmd_msg.data & B0100;
  ROS_V3_Cmd = ROS_Cmd_msg.data & B0010;
  ROS_V4_Cmd = ROS_Cmd_msg.data & B0001;
}

// Instantiate ROS Subscribers
ros::Subscriber<std_msgs::Byte> ROS_Valve_Sub("ROS_Valve_Commands", &ROS_Cmd_Callback);

// Initialize ROS Publishers
std_msgs::Byte ROS_Status_msg;      // ROS Feedback Message

// Instantiate ROS Publishers
ros::Publisher ROS_Valve_Pub("ROS_Valve_Status", &ROS_Status_msg);

// Button input pins:
const int V1_Button_Pin = 3;   // Valve 1 Manual Button Read Pin
const int V2_Button_Pin = 2;   // Valve 2 Manual Button Read Pin
const int V3_Button_Pin = 5;   // Valve 3 Manual Button Read Pin
const int V4_Button_Pin = 4;   // Valve 4 Manual Button Read Pin

// Transistor base pins:
const int V1_Out_Pin = 6;   // Valve 1 Transistor Output Pin
const int V2_Out_Pin = 7;   // Valve 2 Transistor Output Pin
const int V3_Out_Pin = 8;   // Valve 3 Transistor Output Pin
const int V4_Out_Pin = 9;   // Valve 4 Transistor Output Pin

// Button states (0 == OFF, 1 == ON):
bool V1_Button_State = 0;   // Valve 1 Manual Button State
bool V2_Button_State = 0;   // Valve 2 Manual Button State
bool V3_Button_State = 0;   // Valve 3 Manual Button State
bool V4_Button_State = 0;   // Valve 4 Manual Button State

// Transistor output commands (0 == OFF, 1 == ON):
bool V1_Out_Cmd = 0;      // Valve 1 Manual Button State
bool V2_Out_Cmd = 0;      // Valve 2 Manual Button State
bool V3_Out_Cmd = 0;      // Valve 3 Manual Button State
bool V4_Out_Cmd = 0;      // Valve 4 Manual Button State

// Output byte msg variable:
byte Output_msg = 0;

void setup() {

  // Serial monitor initialization for debugging
  Serial.begin(9600);

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

  // Button pin inputs:
  pinMode(V1_Out_Pin, OUTPUT);
  pinMode(V2_Out_Pin, OUTPUT);
  pinMode(V3_Out_Pin, OUTPUT);
  pinMode(V4_Out_Pin, OUTPUT);
}

void loop() {

  // Update button states
  V1_Button_State = digitalRead(V1_Button_Pin);
  V2_Button_State = digitalRead(V2_Button_Pin);
  V3_Button_State = digitalRead(V3_Button_Pin);
  V4_Button_State = digitalRead(V4_Button_Pin);

  // Debugging: print button states to serial monitor
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
  Serial.println(V4_Button_State);

  // Reset the output message:
  Output_msg = 0;

  // Update Valve 1 Command
  if (V1_Button_State == HIGH || ROS_V1_Cmd == HIGH) {
    digitalWrite(V1_Out_Pin, HIGH);
    Output_msg = Output_msg | B1000;
  } else {
    digitalWrite(V1_Out_Pin, LOW);
  }

  // Update Valve 2 Command
  if (V2_Button_State == HIGH || ROS_V2_Cmd == HIGH) {
    digitalWrite(V2_Out_Pin, HIGH);
    Output_msg = Output_msg | B0100;
  } else {
    digitalWrite(V2_Out_Pin, LOW);
  }

  // Update Valve 3 Command
  if (V3_Button_State == HIGH || ROS_V3_Cmd == HIGH) {
    digitalWrite(V3_Out_Pin, HIGH);
    Output_msg = Output_msg | B0010;
  } else {
    digitalWrite(V3_Out_Pin, LOW);
  }

  // Update Valve 4 Command
  if (V4_Button_State == HIGH || ROS_V4_Cmd == HIGH) {
    digitalWrite(V4_Out_Pin, HIGH);
    Output_msg = Output_msg | B0001;
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
