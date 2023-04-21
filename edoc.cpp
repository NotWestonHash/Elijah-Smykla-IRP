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
distance Dyst = distance(PORT8);
motor armMotorA = motor(PORT5, true);
motor armMotorB = motor(PORT11, false);
motor_group arm = motor_group(armMotorA, armMotorB);




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
double spinV=1;//rightjoy
double theta;
double xmm=5000;
double ymm=5000;
//give map dm res
int map[100][100];//resolution of cm
double tiempo=0;
int cycles=0;
double xoff=-140;
double yoff=140;
double occidentX=0;
double occidentY=0;
double occipitalX=0;
double occipitalY=0;
double icoordX=0;
double icoordY=0;
double start=0;





//sets up motors
void motorset(){
  //Stoptypes
  FD.setStopping(brake);
  LD.setStopping(brake);
  BD.setStopping(brake);
  RD.setStopping(brake);
  arm.setStopping(hold);
  
  FD.setMaxTorque(100,percent);
  LD.setMaxTorque(100,percent);
  BD.setMaxTorque(100,percent);
  RD.setMaxTorque(100,percent);
  Arm.setMaxTorque(100,percent);
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




//angle finder that works better than atan
double arctangent(double y, double x){
  if(x<0){
    return atan(y/x)+M_PI;}
    else if(x==0){
      if(y>0){
        return M_PI/2;
      }
      else{
        return M_PI/-2;
      }
    }
    else{return atan(y/x);
  }
}




//returns greatest of raw motor outputs
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




//if near a bad point
/*
bool obstacle(double x,double y){
  for(int fx=x-2;fx<=x+2;fx++){
    for(int fy=y-2;fy<=y+2;fy++){
      if(fx<0||fx>100||fy<0||fy>100){
        return true;
      }
      if(map[fx][fy]!=0){
        return true;
      }
    }
  }
  return false;
}
*/








int main() {

  //calibrating sensors and motors
  motorset();
  BrainInertial.calibrate();
  wait(5,seconds);

  //awaiting startup
  while(!Controller.ButtonFUp.pressing()){
    wait(1,msec);
  }

  //setting rotation
  BrainInertial.setHeading(0, degrees);
  BrainInertial.setRotation(0, degrees);
  start=Brain.Timer.value();
  //main loop
  while(true){

    //gets input angle plus offset
    theta=arctangent(axisA,axisB)+((M_PI*axisC)/(180*(100/(100-49.025)))); 

    //checks map vincinity
    if(/*obstacle((int)(xmm/100),(int)(ymm/100))*/Dist.objectDistance(mm)<100||Dyst.objectDistance(mm)<100){
      //stops until fup press
      FD.stop();
      BD.stop();
      RD.stop();
      LD.stop();
      while(!Controller.ButtonFUp.pressing()){
        wait(1,msec);
      }
      FD.setVelocity(0,percent);
      LD.setVelocity(0,percent);
      BD.setVelocity(0,percent);
      RD.setVelocity(0,percent);
      FD.spin(forward);
      LD.spin(forward);
      BD.spin(forward);
      RD.spin(forward);
    }

    //sets start of cycle
    tiempo=Brain.Timer.value();

    //sets axis readings as smaller vars
    axisA=Controller.AxisA.position();
    axisB=Controller.AxisB.position();
    axisC=Controller.AxisC.position();
    axisD=Controller.AxisD.position();

    //sets reoriented x and y motor output values
    orientX=sqrt(pow(axisB,2)+pow(axisA,2)) * cos(theta+(M_PI*(BrainInertial.orientation(yaw,degrees)/180)));
    orientY=sqrt(pow(axisB,2)+pow(axisA,2)) * sin(theta+(M_PI*(BrainInertial.orientation(yaw,degrees)/180)));
    
    //proportionalizes the motor outputs
    if(greatest(orientX,orientY,axisC)>100){
      spinV=(100/greatest(orientX,orientY,axisC));
    }
    else{
      spinV=1;
    }

    //setVelocity
    FD.setVelocity((-orientX-axisC)*spinV,percent);
    BD.setVelocity((orientX-axisC)*spinV,percent);
    RD.setVelocity((orientY-axisC)*spinV,percent);
    LD.setVelocity((-orientY-axisC)*spinV,percent);
    arm.setVelocity((int)(axisD/2));
    //cycle update
    cycles++;

    //rpmp%=1.27
    //rpsp%=1.27/60
    //circ=200
    //mm=circ*rpsp%*Δts*%out

    //sets coords, and makes intermittant noises
      xmm+=(200*(1.27/2840)*(Brain.Timer.value()-tiempo)*((axisB)*spinV));//accounts for gears
      ymm+=(200*(1.27/2840)*(Brain.Timer.value()-tiempo)*((axisA)*spinV));
      Brain.playNote(1+((cycles/7)%8),cycles%7,1);//A really annoying feature

    //creates offset vector components for sensors-should be added to the sight vector to get cartesian sight coords
    occidentX=sqrt(pow(xoff,2)+pow(yoff,2)) * cos(arctangent(yoff,xoff)+(M_PI*(BrainInertial.orientation(yaw,degrees)/180)));
    occidentY=sqrt(pow(xoff,2)+pow(yoff,2)) * sin(arctangent(yoff,xoff)+(M_PI*(BrainInertial.orientation(yaw,degrees)/180)));
    
    //plots obstacles
    if(Dist.isObjectDetected()){

      //splits sight vector into components
      occipitalX=Dist.objectDistance(mm)*cos(M_PI*(BrainInertial.orientation(yaw,degrees))/180);
      occipitalY=Dist.objectDistance(mm)*sin(M_PI*(BrainInertial.orientation(yaw,degrees))/180);

      //gets coordinates of the obstacle
      icoordX=occipitalX+occidentX+xmm;
      icoordY=occipitalY+occidentY+ymm;

      //if inbounds, plot
      if(!(icoordX>10000||icoordX<0||icoordY>10000||icoordY<0)){
        map[(int)(icoordX/100)][(int)(icoordY/100)]=1;
      }
    }

    if(Dyst.isObjectDetected()){

      //splits sight vector into components
      occipitalX=-1*Dyst.objectDistance(mm)*cos(M_PI*(BrainInertial.orientation(yaw,degrees))/180);
      occipitalY=-1*Dyst.objectDistance(mm)*sin(M_PI*(BrainInertial.orientation(yaw,degrees))/180);
      
      //gets coordinates of the obstacle
      icoordX=occipitalX-occidentX+xmm;
      icoordY=occipitalY-occidentY+ymm;

      //if inbounds, plot
      if(!(icoordX>10000||icoordX<0||icoordY>10000||icoordY<0)){
        map[(int)(icoordX/100)][(int)(icoordY/100)]=1;
      }
    }
    Brain.Screen.print("%f",xmm);
    Brain.Screen.newLine();
  }
}
