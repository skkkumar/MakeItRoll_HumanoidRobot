/* 
 * Copyright (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Sriram Kumar
 * email: sriram.kishore@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/
/*
For icub to look at the blue ball down and push it

Copyright (C) 2010 RobotCub Consortium
 
Author: Sriram Kumar 

CopyPolicy: Released under the terms of the GNU GPL v2.0. 

Scenario : 
To write a simple yarp code that does the following things: 

1. Make iCub look down to the table

2. Detect the blue ball in both images planes

3. Retrieve the ball position in the Cartesian domain

4. Ask iCub to reach for the ball and make it roll!
*/

#include <string>
#include <stdio.h>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/math.h>
YARP_DECLARE_DEVICES(icubmod)
using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;

/***************************************************/
class CtrlModule: public RFModule
{
protected:
  PolyDriver drvArm, drvGaze, robotMotor;
  ICartesianControl *iarm;
  IGazeControl      *igaze;
  
  BufferedPort<ImageOf<PixelRgb> > imgLPortIn,imgRPortIn;
  Port imgLPortOut,imgRPortOut;
  RpcServer rpcPort;
  
  Mutex mutex;
  Vector cogL,cogR;
  bool okL,okR;
  string side;
  int startup_context_id;
  /***************************************************/
  bool getCOG(ImageOf<PixelRgb> &img, Vector &cog)
  {
    int xMean=0;
    int yMean=0;
    int ct=0;
    
    for (int x=0; x<img.width(); x++)
    {
      for (int y=0; y<img.height(); y++)
      {
	PixelRgb &pixel=img.pixel(x,y);
	if ((pixel.b>5.0*pixel.r) && (pixel.b>5.0*pixel.g))
	{
	  xMean+=x;
	  yMean+=y;
	  ct++;
	}
      }
    }
    
    if (ct>0)
    {
      cog.resize(2);
      cog[0]=xMean/ct;
      cog[1]=yMean/ct;
      return true;
    }
    else
      return false;
  }
  
  /***************************************************/
  Vector retrieveTarget3D(const Vector &cogL, const Vector &cogR)
  {
    // FILL IN THE CODE
    Vector codvct;
    int startup_context_id;
    drvGaze.view(igaze);
    igaze->storeContext(&startup_context_id);
    if (cogL[0] < cogR[0]){  // blue ball is located near the left arm
      igaze->get3DPoint(0,cogL,0.4,codvct);
      side = "left";
    }
    else{  // blue ball is located near the right arm
      igaze->get3DPoint(1,cogR,0.4,codvct);
      side = "right";
    }
    igaze->stopControl();
    igaze->restoreContext(startup_context_id);
    return codvct;
  }
  
  /***************************************************/
  void fixate(Vector &x)
  {
    if (side == "left"){
      x[0] += 0.05;
      x[1] -= 0.08;
//       x[2] -= 0.01; 
      
    }
    else{
      x[0] += 0.05;
      x[1] += 0.08;
//       x[2] -= 0.01;
      
    }
  }
  
  
  /***************************************************/
  Vector computeHandOrientation()
  {
    Vector od;
    od.resize(4);
    od[0]=0.0; od[1]=0.0; od[2]=1.0; od[3]=3.0;	
    //     if (side == "left"){
    //       od[0]=-1.0; od[1]=0.0; od[2]=-0.0; od[3]=3.0;	  
    //     }
    //     else{
    //       od[0]=-0.03; od[1]=-0.7; od[2]=0.7; od[3]=3.0;
    //     }
    return od;
    
  }
  
  /***************************************************/
  // To start different Arm with respect to the use
  bool cartesianMotordevice(string contLocation){
    
    drvArm.close();
    Property option("(device cartesiancontrollerclient)");
    option.put("remote",("/icubSim/cartesianController/"+contLocation+"").c_str());
    option.put("local",("/cartesian_client/"+contLocation+"").c_str());
    
    if (!drvArm.open(option))
      return false;
    return true;
  }
  
  int approachTargetWithHand(const Vector &x, const Vector &o)
  {
    if (side == "left"){
      if (!cartesianMotordevice("left_arm")){
	return 1;
      }	  
    }
    else{
      if (!cartesianMotordevice("right_arm")){
	return 1;
      }	  
    }
    
    drvArm.view(iarm);
    iarm->storeContext(&startup_context_id);
    iarm->setTrajTime(1.0);
    iarm->goToPose(x,o);
    if (iarm->waitMotionDone(2)) {
      printf("Problem occured in moving the hand to position");
    }
    iarm->stopControl();
    iarm->restoreContext(startup_context_id);
  }
  
  
  /***************************************************/
  void makeItRoll(Vector &x, const Vector &o)
  {
    drvArm.view(iarm);
    iarm->storeContext(&startup_context_id);
    iarm->setTrajTime(1.0);
//     x[0] = x[0] * -1;
    x[1] = x[1] * -1;
//     x[2] = x[2] * -1;
    iarm->goToPose(x,o);
    if (iarm->waitMotionDone(2)) {
      printf("Problem occured in Rolling the ball with hand");
    }
    iarm->stopControl();
    iarm->restoreContext(startup_context_id);
  }
  
  /***************************************************/
  
  int move_eye_down(){
    if (!startMotordevice("head")){
      return 1;
    }
    IPositionControl *pos;
    IVelocityControl *vel;
    IEncoders *enc;
    robotMotor.view(pos);
    robotMotor.view(vel);
    robotMotor.view(enc);
    if (pos==NULL || vel==NULL || enc==NULL) 
    {
      printf("Cannot get interface to robot head\n");
      robotMotor.close();
      return 1;
    }
    int jnts = 0;
    pos->getAxes(&jnts);
    Vector setpoints;
    setpoints.resize(jnts);
    for (int i=0; i<jnts; i++) 
    {
      setpoints[i] = 0;
    }
    pos->positionMove(setpoints.data());
    yarp::os::Time::delay(2);
    setpoints[2] = 0;
    setpoints[3] = -30;
    pos->positionMove(setpoints.data());
    yarp::os::Time::delay(2);
    
    
  }
  
  void gaze_ball()
  {
    drvGaze.view(igaze);
    igaze->storeContext(&startup_context_id);
    igaze->setNeckTrajTime(0.8);
    igaze->setEyesTrajTime(0.4);
    igaze->lookAtStereoPixels(cogL,cogR);
    if (igaze->waitMotionDone(2)){
      igaze->stopControl();
      igaze->restoreContext(startup_context_id);
    }
  }
  
  void look_down()
  {
    move_eye_down();
    gaze_ball();
  }
  
  /***************************************************/
  void roll(const Vector &cogL, const Vector &cogR)
  {
    printf("detected cogs = (%s) (%s)\n",
	   cogL.toString(0,0).c_str(),cogR.toString(0,0).c_str());
    
    Vector x=retrieveTarget3D(cogL,cogR);
    printf("retrieved 3D point = (%s)\n",x.toString(3,3).c_str());
    printf("Arm used = (%s)\n",side.c_str()); 
    
    fixate(x);
    printf("fixating at (%s)\n",x.toString(3,3).c_str());
    
    Vector o=computeHandOrientation();
    printf("computed orientation = (%s)\n",o.toString(3,3).c_str());
    
    approachTargetWithHand(x,o);
    printf("approached\n");
    
    makeItRoll(x,o);
    printf("roll!\n");
  }
  
  /***************************************************/
  int home_head()
  {
    if (!startMotordevice("head")){
      return 1;
    }
    IPositionControl *pos;
    robotMotor.view(pos);
    if (pos==NULL) 
    {
      printf("Cannot get interface to robot head\n");
      robotMotor.close();
      return 1;
    }
    int jnts = 0;
    pos->getAxes(&jnts);
    Vector setpoints;
    setpoints.resize(jnts);
    for (int i=0; i<jnts; i++) 
    {
      setpoints[i] = 0;
    }
    pos->positionMove(setpoints.data());
    yarp::os::Time::delay(2);
    //     vel->velocityMove(setpoints.data());
  }
  
  
  int home_leftArm()
  {
    if (!startMotordevice("left_arm")){
      return 1;
    }
    IPositionControl *pos;
    robotMotor.view(pos);
    if (pos==NULL) 
    {
      printf("Cannot get interface to robot head\n");
      robotMotor.close();
      return 1;
    }
    int jnts = 0;
    pos->getAxes(&jnts);
    Vector setpoints;
    setpoints.resize(jnts);
    for (int i=0; i<7; i++) 
    {
      setpoints[i] = 0;
    }
    setpoints[1] = 80;
    setpoints[3] = 50;
    setpoints[7] = 59;
    for (int i=8; i<11; i++) 
    {
      setpoints[i] = 0;
    }
    for (int i=11; i<jnts; i++) 
    {
      setpoints[i] = 0;
    }
    pos->positionMove(setpoints.data());
    yarp::os::Time::delay(2);
    //     vel->velocityMove(setpoints.data());
  }
  
  int home_rightArm()
  {
    if (!startMotordevice("right_arm")){
      return 1;
    }
    IPositionControl *pos;
    robotMotor.view(pos);
    if (pos==NULL) 
    {
      printf("Cannot get interface to robot head\n");
      robotMotor.close();
      return 1;
    }
    int jnts = 0;
    pos->getAxes(&jnts);
    Vector setpoints;
    setpoints.resize(jnts);
    for (int i=0; i<7; i++) 
    {
      setpoints[i] = 0;
    }
    setpoints[1] = 80;
    setpoints[3] = 50;
    setpoints[7] = 59;
    for (int i=8; i<11; i++) 
    {
      setpoints[i] = 0;
    }
    for (int i=11; i<jnts; i++) 
    {
      setpoints[i] = 0;
    }
    pos->positionMove(setpoints.data());
    yarp::os::Time::delay(2);
    //     vel->velocityMove(setpoints.data());
  }
  
  
  
  void home()
  {
    home_head();
    home_leftArm();
    home_rightArm();
  }
  
  /***************************************************/ 
  
  bool startMotordevice(string contLocation){
    robotMotor.close();
    Property options;
    options.put("device", "remote_controlboard");
    options.put("local", "/mover/motor/cient");
    options.put("remote", ("/icubSim/"+contLocation+"").c_str());
    if (!robotMotor.open(options)) 
    {
      printf("Cannot connect to robot head\n");
      return false;
    }
    return true;
  }
  
public:
  /***************************************************/
  bool configure(ResourceFinder &rf)
  {
    // FILL IN THE CODE
    
    imgLPortIn.open("/imgL:i");
    imgRPortIn.open("/imgR:i");
    
    imgLPortOut.open("/imgL:o");
    imgRPortOut.open("/imgR:o");
    
    rpcPort.open("/service");
    attach(rpcPort);
    
    string name=rf.find("name").asString().c_str();
    
    // 	options.put("remote", "/icubSim/head");
    // 	if (!robotHead.open(options)) 
    // 	{
    // 	    printf("Cannot connect to robot head\n");
    // 	    return false;
    // 	}
    // 	options.clear();
    
    Property optGaze("(device gazecontrollerclient)");
    optGaze.put("remote","/iKinGazeCtrl");
    optGaze.put("local",("/"+name+"/gaze").c_str());
    if (!drvGaze.open(optGaze))
      return false;
    
    return true;
  }
  
  /***************************************************/
  bool interruptModule()
  {
    imgLPortIn.interrupt();
    imgRPortIn.interrupt();
    return true;
  }
  
  /***************************************************/
  bool close()
  {
    drvArm.close();
    drvGaze.close();
    robotMotor.close();
    imgLPortIn.close();
    imgRPortIn.close();
    imgLPortOut.close();
    imgRPortOut.close();
    rpcPort.close();
    return true;
  }
  
  /***************************************************/
  bool respond(const Bottle &command, Bottle &reply)
  {
    string cmd=command.get(0).asString().c_str();
    if (cmd=="look_down")
    {
      look_down();
      reply.addString("Yep! I'm looking down now!");
      return true;
    }
    else if (cmd=="roll")
    {
      if (cogL[0] || cogR[0])
      {
	roll(cogL,cogR);
	reply.addString("Yeah! I've made it roll like a charm!");
      }
      else
	reply.addString("I don't see any object!");
      
      return true;
    }
    else if (cmd=="home")
    {
      home();
      reply.addString("I've got the hard work done! Going home.");
      return true;
    }
    else
      return RFModule::respond(command,reply);
  }
  
  /***************************************************/
  double getPeriod()
  {
    return 0.0;     // sync upon incoming images
  }
  
  /***************************************************/
  bool updateModule()
  {
    // get fresh images
    ImageOf<PixelRgb> *imgL=imgLPortIn.read();
    ImageOf<PixelRgb> *imgR=imgRPortIn.read();
    
    // interrupt sequence detected
    if ((imgL==NULL) || (imgR==NULL))
      return false;
    
    // compute the center-of-mass of pixels of our color
    mutex.lock();
    okL=getCOG(*imgL,cogL);
    okR=getCOG(*imgR,cogR);
    mutex.unlock();
    
    PixelRgb color;
    color.r=255; color.g=0; color.b=0;
    
    if (okL)
      draw::addCircle(*imgL,color,(int)cogL[0],(int)cogL[1],5);
    
    if (okR)
      draw::addCircle(*imgR,color,(int)cogR[0],(int)cogR[1],5);
    
    imgLPortOut.write(*imgL);
    imgRPortOut.write(*imgR);
    
    return true; 
  }
};


/***************************************************/
int main(int argc, char *argv[])
{   
  
  
  Network yarp;
  if (!yarp.checkNetwork())
    return -1;
  
  CtrlModule mod;
  YARP_REGISTER_DEVICES(icubmod)
  ResourceFinder rf;
  rf.setVerbose(true);
  rf.setDefault("name","tracker");
  //     rf.setDefault("period","0.02");
  
  rf.configure(argc,argv);
  return mod.runModule(rf);
}



