#include <Arduino.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt16.h>

// This file is created in2020 by YuanHaobo  <yhb0521@126.com>

ros::NodeHandle  nh;

Servo upservo; //4th DOF, upper one
Servo lowservo;//5th DOF, lower one

void servo_cb( const std_msgs::UInt16& cmd_msg){
  upservo.write(cmd_msg.data); //set servo angle, should be from 0-180   
}


ros::Subscriber<std_msgs::UInt16> sub("servo", servo_cb);


void setup() {
 
  Serial.begin(9600);
  pinMode(13,OUTPUT);//LED, for knowing the voltage status
  pinMode(A13,INPUT);//uarm

  nh.initNode();
  nh.subscribe(sub);
  
  upservo.attach(2);//4th DOF
  lowservo.attach(3);//5th DOF
}




void loop() {

  int gripper=analogRead(A13);//A13 is connect to uarm
  Serial.println(gripper);//for knowing the voltage status
  
  if(gripper>800){//default is 0,or High voltage level
    digitalWrite(13,HIGH);//turn on LED
    lowservo.write(0);//default is deattach
    
  }else{//status is 1, or low voltage level
    digitalWrite(13,LOW);//turn off LED
    lowservo.write(50);//attach
  }

  nh.spinOnce();
  delay(1000);

  
}
