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
motor FD = motor(PORT12, false);
motor RD = motor(PORT7, false);
motor LD = motor(PORT6, false);
motor BD = motor(PORT1, false);
controller Controller = controller();



// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
#pragma endregion VEXcode Generated Robot Configuration

#include "vex.h"
using namespace vex;
double axisA=0;//LV
double axisB=0;//LH
double axisC=0;//RV
double axisD=0;//RH
double orientX=0;//postorient
double orientY=0;//postorient
double spinV=0;//rightjoy
double theta;
double xmm=0;
double ymm=0;
//give map dm res
int ydmap[100][100];
double tiempo=0;
int cycles=0;

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

double arctangent(double y, double x){
  if(x<0){
    return atan(y/x)+M_PI;
  }
  else if(x==0){
    if(y>0){return M_PI/2;}
    else{return M_PI/-2;}
  }
  else{
    return atan(y/x);
  }
}
double greatest(double orx,double ory,double spy){
  double bigboy=0.0;
  double ors[4]={-1*orx,orx,-1*ory,ory};
  for(int index=0;index<4;index++){
    if(fabs(ors[index]-spy)>fabs(bigboy)){
      bigboy=ors[index];
    }
  }
  return bigboy;
}

























int main() {
  motorset();
  BrainInertial.calibrate();
  wait(5,seconds);
  
  while(!Controller.ButtonFUp.pressing()){
    wait(1,msec);
  }
  BrainInertial.setHeading(0, degrees);
  BrainInertial.setRotation(0, degrees);
  while(true){
    tiempo=Brain.Timer.value();
    axisA=Controller.AxisA.position();
    axisB=Controller.AxisB.position();
    axisC=Controller.AxisC.position();
    axisD=Controller.AxisD.position();
    spinV=axisD;//BXAYCR
    theta=arctangent(axisA,axisB)+((M_PI*axisC)/(180*(100/(100-49.025)))); 
    
    orientX=sqrt(pow(axisB,2)+pow(axisA,2))/*magnitude*/ * cos(theta+(M_PI*(BrainInertial.orientation(yaw,degrees)/180)));
    orientY=sqrt(pow(axisB,2)+pow(axisA,2))/*magnitude*/ * sin(theta+(M_PI*(BrainInertial.orientation(yaw,degrees)/180)));
    //naive field orient test:
    Brain.Screen.newLine();

    Brain.Screen.print("X: %d",xmm);
    Brain.Screen.print(" ");
    Brain.Screen.print("Y: %d",ymm);
    //orientX=axisB;
    //orientY=axisA;
    if(greatest(orientX,orientY,axisC)>100){
      spinV=(100/greatest(orientX,orientY,axisC));
    }
    else{
      spinV=1;
    }
    FD.setVelocity((-orientX-axisC)*spinV,percent);
    BD.setVelocity((orientX-axisC)*spinV,percent);
    RD.setVelocity((orientY-axisC)*spinV,percent);
    LD.setVelocity((-orientY-axisC)*spinV,percent);
    cycles++;
    xmm+=(254/60)*(Brain.Timer.value()-tiempo)*(orientX*spinV);
    ymm+=(254/60)*(Brain.Timer.value()-tiempo)*(orientY*spinV);
  }
 

}
