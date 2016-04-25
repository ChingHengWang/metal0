/*
  file: sharpIR_Code.ino
  date: 20160321
  description :
        initial code for ros 

  Author: Zach
           qoogood1234@gmail.com
*/

//#include <SharpIR.h>
#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <Arduino.h>
#include <math.h>

#define  SIDE_LEFT_FOLLOW_IR            A0            // Read left following IR sensor pin
#define  SIDE_LEFT_FOLLOW_IR_VALUE     analogRead(SIDE_LEFT_FOLLOW_IR)

#define  LEFT_FOLLOW_IR            A1            // Read left following IR sensor pin
#define  LEFT_FOLLOW_IR_VALUE     analogRead(LEFT_FOLLOW_IR)

#define  FOLLOWIR            A2           // Read Middle following IR sensor pin
#define  FOLLOW_IR_VALUE     analogRead(FOLLOWIR)


#define  RIGHT_FOLLOW_IR           A3            // Read right following IR sensor pin
#define  RIGHT_FOLLOW_IR_VALUE     analogRead(RIGHT_FOLLOW_IR)

#define  SIDE_RIGHT_FOLLOW_IR            A4            // Read left following IR sensor pin
#define  SIDE_RIGHT_FOLLOW_IR_VALUE     analogRead(SIDE_RIGHT_FOLLOW_IR)




float IR_Distance = 0, Left_IR_Distance = 0, Right_IR_Distance = 0, Side_Left_IR_Distance = 0, Side_Right_IR_Distance = 0;
float Follow_IR_f = 0, Left_Follow_IR_f = 0, Right_Follow_IR_f = 0, Side_Left_Follow_IR_f = 0, Side_Right_Follow_IR_f = 0;
unsigned long PastTime = 0, PastPrintTime = 0, PastFilterTime = 0;
unsigned long PresentTime = millis();




ros::NodeHandle nh;
geometry_msgs::Vector3 ir_value;
geometry_msgs::Vector3 ir_value_side;
ros::Publisher pub_ir_value("sensor/ir_value",&ir_value);
ros::Publisher pub_ir_value_side("sensor/ir_value_side",&ir_value_side);

void setup() {
  pinMode(FOLLOWIR, INPUT);  
  pinMode(LEFT_FOLLOW_IR, INPUT);
  pinMode(RIGHT_FOLLOW_IR, INPUT); 
  pinMode(SIDE_LEFT_FOLLOW_IR, INPUT);
  pinMode(SIDE_RIGHT_FOLLOW_IR, INPUT);   
  nh.initNode();
  nh.advertise(pub_ir_value);
  nh.advertise(pub_ir_value_side);
  Serial.begin(57600);
}

void loop() {

  PresentTime = millis();              
  if((int)(PresentTime-PastFilterTime) > 20){   // 20ms
    PastFilterTime = PresentTime;
      // Filter the IR sensor value
    Follow_IR_f = 0.9*Follow_IR_f + 0.1*FOLLOW_IR_VALUE;
    Left_Follow_IR_f = 0.9*Left_Follow_IR_f + 0.1*LEFT_FOLLOW_IR_VALUE;
    Right_Follow_IR_f = 0.9*Right_Follow_IR_f + 0.1*RIGHT_FOLLOW_IR_VALUE;
    Side_Left_Follow_IR_f = 0.9*Side_Left_Follow_IR_f + 0.1*SIDE_LEFT_FOLLOW_IR_VALUE;
    Side_Right_Follow_IR_f = 0.9*Side_Right_Follow_IR_f + 0.1*SIDE_RIGHT_FOLLOW_IR_VALUE;    
    
    IR_Distance = 10650.08 * pow(Follow_IR_f, -0.935) - 10;                // Transform the filtered IR sensor value to distance
    Left_IR_Distance = 10650.08 * pow(Left_Follow_IR_f, -0.935) - 10; 
    Right_IR_Distance = 10650.08 * pow(Right_Follow_IR_f, -0.935) - 10;
    Side_Left_IR_Distance = 10650.08 * pow(Side_Left_Follow_IR_f, -0.935) - 10; 
    Side_Right_IR_Distance = 10650.08 * pow(Side_Right_Follow_IR_f, -0.935) - 10;      
  }

  PresentTime = millis();              // 8~14 us
  if((int)(PresentTime-PastPrintTime) > 100){   // 20ms
    PastPrintTime = PresentTime;
    ir_value_side.x = Side_Left_IR_Distance;
    ir_value.x = Left_IR_Distance;
    ir_value.y = IR_Distance;
    ir_value.z = Right_IR_Distance;
    ir_value_side.y = Side_Right_IR_Distance;    
  
    pub_ir_value.publish(&ir_value);
    pub_ir_value_side.publish(&ir_value_side);    
    nh.spinOnce();
    //printIRInfo(); 
  }

}

void printIRInfo()
{
  Serial.print(PresentTime);
  Serial.println("  ");

  }
