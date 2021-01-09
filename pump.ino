#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include "AutoPID.h"
#include "MotorDrive.h"

// pins
#define pumpA 6
#define pumpB 7
#define pumpPWM 9
#define valveA 4
#define valveB 5
#define valvePWM 8
#define airPin A2
#define irPin 3

// pid settings and gains
// pwm: min: 50 max: 255
#define OUTPUT_MIN -255
#define OUTPUT_MAX 255
#define KP 10000
#define KI 0
#define KD 300

#define PUBLISH_DELAY 1000


// 0 = ready; 1 = sucking; 2 = sucked; 3 = releasing; 4 = dropped; 5 = stop
int stage = 5;
int fail = 0;
int success = 0;

double airpr, setPoint, outputVal;
//input/output variables passed by reference, so they are updated automatically
AutoPID myPID(&airpr, &setPoint, &outputVal, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

unsigned long lastPublish;

// ROS
ros::NodeHandle nh;
void suctionCb( const std_msgs::String& sucker_cmd){
  // format: on/off + space + [strength]
  String cmd = sucker_cmd.data;
  int on = cmd.substring(0, 1).toInt();
  if(on == 1){
    updateStage(1);
    setPoint = cmd.substring(2).toDouble();
  }else{
    updateStage(3);
  }
}



std_msgs::String str_msg;
std_msgs::Float32 str_msg2;
ros::Publisher chatter("pump_state", &str_msg);
ros::Publisher chatter2("pressure_level", &str_msg2);
ros::Subscriber<std_msgs::String> sub("suction_cmd", &suctionCb);


void updateStage(int new_stage){
  if(new_stage != stage){
    if(new_stage == 0){
      str_msg.data = "G0";
    }else if(new_stage == 1){
      str_msg.data = "G1";
    }else if(new_stage == 2){
      str_msg.data = "G2";
    }else if(new_stage == 3){
      str_msg.data = "G3";
    }else if(new_stage == 4){
      str_msg.data = "G4";
    }else if(new_stage == 5){
      str_msg.data = "G5";
    }
    
    chatter.publish(&str_msg);
    stage = new_stage;
  }
}
//call repeatedly in loop, only updates after a certain time interval
//returns true if update happened
bool updatePressure() {
    
    airpr = analogRead(airPin) * (5.0 / 1023.0);    
    //Serial.println(airpr);
    str_msg2.data = airpr;
    chatter2.publish(&str_msg2);

    // min: 0.0 kPa ~= 1.6V;  max: -83 kPa ~= 4.82V
    // map 0-5 to 1.6-4.8

    //return true;
  //}
  return false;
}//void updatePressure


MotorDrive pump(pumpA, pumpB, pumpPWM);
MotorDrive valve(valveA, valveB, valvePWM);
int pwmMin = 50;
void setup() {
  Serial.begin(57600);
  /*
  pump.MoveBackward(255);
  delay(10000);
  pump.MoveBackward(0);
  delay(2000);
  valve.MoveBackward(120);
  delay(5000);
  valve.MoveBackward(0);
  */
  nh.subscribe(sub);
  nh.advertise(chatter);
  nh.advertise(chatter2);
  str_msg.data = "G0";
  setPoint = 2;
  if(setPoint < 2){
     pwmMin = 70;
  }else if(setPoint < 2.4){
    pwmMin = 100;
  }else if(setPoint < 3){
    pwmMin = 140;
  }else{
    pwmMin = 150;
  }

  updateStage(0);
}


void loop() {
  if(stage == 0){
    if((millis() - lastPublish) > PUBLISH_DELAY){
      str_msg.data = "G0";
      chatter.publish(&str_msg);
      lastPublish = millis();
    }
      
  }else if(stage == 1){

    // failed if analog value is always smaller than 2.0 
    if(airpr > setPoint){
      success ++;
      fail = 0;
    }else{
      fail ++;
      success = 0;
    }
    
    if(success > 20){
      updateStage(2);
    }else if(fail > 50000){
      updateStage(0);
      // send fail msg to pc
    }
    
  }else if(stage == 2){
    // do nothing
    // wait for release cmd from pc
  }else if(stage == 3){
    if(airpr < 1.7){
      updateStage(4);
    }
  }else if(stage == 4){
    // return to ready stage
    updateStage(0);
  }

  if(stage == 1 || stage == 2){
    //updatePressure();
    // read new setPoint
  
    myPID.run(); //call every loop, updates automatically at certain time interval

    int pwmVal = (int)outputVal;
    if(pwmVal < 0){
      pwmVal = 0;
    }else{
      map(pwmVal, -255, 255, pwmMin, 255);
    }
    pump.MoveBackward(pwmVal);
    //Serial.println("air pre: ");
    //Serial.println(outputVal);
    //Serial.println("set point: ");
    //Serial.println(pwmVal);   
  }else if(stage == 3){
    // stop pid and pump
    myPID.stop();
    pump.MoveBackward(0);
    // release pressure
    valve.MoveBackward(255);

  }else if(stage == 4){
    valve.MoveBackward(0);
  }

  updatePressure();

  nh.spinOnce();
  delay(10);

/*
  if((millis() - lastPublish) > PUBLISH_DELAY){
    lastPublish = millis();
    chatter.publish(&str_msg);
  }
*/
}
