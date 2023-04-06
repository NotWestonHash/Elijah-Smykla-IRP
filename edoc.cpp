#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>


#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;


// START IQ MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END IQ MACROS


// Robot configuration code.
inertial BrainInertial = inertial();
distance Dist = distance(PORT2);
motor FD = motor(PORT1, false);
motor RD = motor(PORT6, false);
motor LD = motor(PORT7, false);
motor BD = motor(PORT12, false);
controller Controller = controller();



// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
#pragma endregion VEXcode Generated Robot Configuration

#include "vex.h"
using namespace vex;
int axisA=0;//LV
int axisB=0;//LH
int axisC=0;//RV
int axisD=0;//RH
int orientX=0;//postorient
int orientY=0;//postorient
int spinV=0;//rightjoy
int theta=0;



// Create pathfinding as something done by the robot on keystroke

// setMaxTorque(100,percent);

void motorset(){
  //Stoptypes
  FD.setStopping(brake);
  LD.setStopping(brake);
  BD.setStopping(brake);
  RD.setStopping(brake);
  //RG.setStopping(brake);
  //LG.setStopping(brake);
  //Load.setStopping(brake);
  //Alt.setStopping(hold);
  //Torquesets
  FD.setMaxTorque(100,percent);
  LD.setMaxTorque(100,percent);
  BD.setMaxTorque(100,percent);
  RD.setMaxTorque(100,percent);
  //RG.setMaxTorque(100,percent);
  //LG.setMaxTorque(100,percent);
  //Load.setMaxTorque(100,percent);
  //Alt.setMaxTorque(100,percent);
  //Novelocity
  FD.setVelocity(0,percent);
  LD.setVelocity(0,percent);
  BD.setVelocity(0,percent);
  RD.setVelocity(0,percent);
  //Spinmotors
  FD.spin(forward);
  LD.spin(forward);
  BD.spin(forward);
  RD.spin(forward);
}

void fupcalibrater(){
  while(true){
    
  }
}

int main() {
  motorset();
  BrainInertial.calibrate();
  while(!Controller.ButtonFUp.pressing()){
    wait(10,msec);
  }
  BrainInertial.setHeading(0, degrees);
  BrainInertial.setRotation(0, degrees);
  while(true){
    axisA=Controller.AxisA.position();
    axisB=Controller.AxisB.position();
    axisC=Controller.AxisC.position();
    axisD=Controller.AxisD.position();
    spinV=axisD;//BXAY
    //0/0 witin atan.
    //use some logic to better determine θ
    if(axisB==0){
      theta=0;
    }
    else{
      theta=atan(axisA/axisB);  
    }
    orientX=sqrt(pow(axisB,2)+pow(axisA,2))*cos(theta-(M_PI*(BrainInertial.orientation(yaw,degrees)/180)));
    orientY=sqrt(pow(axisB,2)+pow(axisA,2))*sin(theta-(M_PI*(BrainInertial.orientation(yaw,degrees)/180)));
    //naive field orient test:
    Brain.Screen.newLine();

    Brain.Screen.print("%f",BrainInertial.orientation(yaw,degrees));

    FD.setVelocity(orientX,percent);
    BD.setVelocity(-orientX,percent);
    RD.setVelocity(-orientY,percent);
    LD.setVelocity(orientY,percent);
    if(Controller.ButtonRUp.pressing()){
      FD.setVelocity(100,percent);
      BD.setVelocity(100,percent);
      RD.setVelocity(100,percent);
      LD.setVelocity(100,percent);
      wait(10,msec);
    }
  }

}
