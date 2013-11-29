/* --------------------------------------------------------------------------
 * Shoulder Elbow to Bioloid
 * --------------------------------------------------------------------------
 * http://code.google.com/p/simple-openni
 * --------------------------------------------------------------------------
 */

import SimpleOpenNI.*;
import processing.serial.*;

/********** Drawing Window **********/
final color[] userClr = new color[] { 
  color(255, 0, 0), 
  color(0, 200, 200), 
  color(0, 0, 255), 
  color(255, 255, 0), 
  color(255, 0, 255), 
  color(0, 255, 255)
};

/********** Kinect **********/
SimpleOpenNI  context;

/********** Arduino **********/
// Assumed to be the first Serial port in Serial.list()
boolean isArduinoConnected;
Serial arduinoSerialPort;

PVector com = new PVector();                                   
PVector com2d = new PVector();                                   

float oldShoulder = 0;
float oldShoulderFlap = 0;
float oldElbow = 0;
float filteredShoulder = 0;
float filteredShoulderFlap = 0;
float filteredElbow = 0;

/********** setup() **********/
void setup() {
  // Set up Arduino
  isArduinoConnected = Serial.list().length > 0;
  if (isArduinoConnected) {
    String portName = Serial.list()[0];
    arduinoSerialPort = new Serial(this, portName, 9600);
    println("Arduino Serial Port Found: " + portName);
  } else {
    println("No Arduino Serial Port Found");
  }

  // Set up Kinect
  context = new SimpleOpenNI(this);
  if (context.isInit() == false) {
    println("Can't init SimpleOpenNI, maybe the camera is not connected!"); 
    exit();
    return;
  }
  // enable depthMap generation 
  context.enableDepth();
  // enable skeleton generation for all joints
  context.enableUser();

  // Set up Drawing Window
  size(640, 480);
  background(200, 0, 0);
  stroke(0, 0, 255);
  strokeWeight(3);
  smooth();
}

/********** draw() **********/
void draw() {
  // update the cam
  context.update();
  // draw depthImageMap
  image(context.depthImage(), 0, 0);
  image(context.userImage(), 0, 0);  
  // draw the skeleton if it's available
  int[] userList = context.getUsers();
  for (int i=0;i<userList.length;i++) {
    if (context.isTrackingSkeleton(userList[i])) {
      stroke(userClr[ (userList[i] - 1) % userClr.length ] );
      drawSkeleton(userList[i]);
    }      
    // draw the center of mass
    if (context.getCoM(userList[i], com)) {
      context.convertRealWorldToProjective(com, com2d);
      stroke(100, 255, 0);
      strokeWeight(1);
      beginShape(LINES);
      vertex(com2d.x, com2d.y - 5);
      vertex(com2d.x, com2d.y + 5);
      vertex(com2d.x - 5, com2d.y);
      vertex(com2d.x + 5, com2d.y);
      endShape();
      fill(0, 255, 100);
      text(Integer.toString(userList[i]), com2d.x, com2d.y);
    }
  }
  
  /* output joint angles. */
  if (userList.length > 0) {
    if (context.isTrackingSkeleton(userList[0])) {
      // output, right now only send values by the 100
      println("Arduino Value");
      // sendValue1 is shoulder 
      float shoulder = shoulderAngle(userList[0]);
      float shoulderFlap = shoulderFlapAngle(userList[0]);
      float elbow = elbowAngle(userList[0]);
      filteredShoulder = 0.99 * oldShoulder + 0.01 * shoulder;
      filteredShoulderFlap = 0.99 * oldShoulderFlap + 0.01 * shoulderFlap;
      filteredElbow = 0.99 * oldElbow + 0.01 * elbow;
      oldShoulder = shoulder;
      oldShoulderFlap = shoulderFlap;
      oldElbow = elbow;
      //println("shoulder angle " + filteredShoulder);
      //println("shoulder flap " + filteredShoulderFlap);
      //println("elbow " + filteredElbow);
      //flapDotProduct(userList[0]);
      println(toAx12(filteredShoulder));
      println(toAx12(filteredShoulderFlap));
      
      // Send to Arduino
      if (isArduinoConnected) {
        arduinoSerialPort.write(toAx12(filteredShoulder));
        arduinoSerialPort.write(toAx12(filteredShoulderFlap));
        arduinoSerialPort.write('\n');
      }
    }
  }
}
/** Helper to convert angle into Ax12 input. */
int toAx12(float inp) {
  return ((int) (500 + inp / (3.14/2) * 300) / 10) + 1;
}

// draw the skeleton with the selected joints
void drawSkeleton(int userId) {
  print("user id ");
  println(userId );
  /* Angle Calculation. */
  context.drawLimb(userId, SimpleOpenNI.SKEL_HEAD, SimpleOpenNI.SKEL_NECK);
  context.drawLimb(userId, SimpleOpenNI.SKEL_NECK, SimpleOpenNI.SKEL_LEFT_SHOULDER);
  context.drawLimb(userId, SimpleOpenNI.SKEL_LEFT_SHOULDER, SimpleOpenNI.SKEL_LEFT_ELBOW);
  context.drawLimb(userId, SimpleOpenNI.SKEL_LEFT_ELBOW, SimpleOpenNI.SKEL_LEFT_HAND);

  context.drawLimb(userId, SimpleOpenNI.SKEL_NECK, SimpleOpenNI.SKEL_RIGHT_SHOULDER);
  context.drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_SHOULDER, SimpleOpenNI.SKEL_RIGHT_ELBOW);
  context.drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_ELBOW, SimpleOpenNI.SKEL_RIGHT_HAND);

  context.drawLimb(userId, SimpleOpenNI.SKEL_LEFT_SHOULDER, SimpleOpenNI.SKEL_TORSO);
  context.drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_SHOULDER, SimpleOpenNI.SKEL_TORSO);

  context.drawLimb(userId, SimpleOpenNI.SKEL_TORSO, SimpleOpenNI.SKEL_LEFT_HIP);
  context.drawLimb(userId, SimpleOpenNI.SKEL_LEFT_HIP, SimpleOpenNI.SKEL_LEFT_KNEE);
  context.drawLimb(userId, SimpleOpenNI.SKEL_LEFT_KNEE, SimpleOpenNI.SKEL_LEFT_FOOT);

  context.drawLimb(userId, SimpleOpenNI.SKEL_TORSO, SimpleOpenNI.SKEL_RIGHT_HIP);
  context.drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_HIP, SimpleOpenNI.SKEL_RIGHT_KNEE);
  context.drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_KNEE, SimpleOpenNI.SKEL_RIGHT_FOOT);
}

/* Helper to calculate angle between 2 vectors. */
float angle(PVector a, PVector b, PVector c) {
  float angle01 = atan2(a.y - b.y, a.x - b.x);
  float angle02 = atan2(b.y - c.y, b.x - c.x);
  float ang = angle02 - angle01;
  return ang;
}

// -----------------------------------------------------------------
// SimpleOpenNI events

void onNewUser(SimpleOpenNI curContext, int userId) {
  println("onNewUser - userId: " + userId);
  println("\tstart tracking skeleton");
  curContext.startTrackingSkeleton(userId);
}

void onLostUser(SimpleOpenNI curContext, int userId) {
  println("onLostUser - userId: " + userId);
}

void onVisibleUser(SimpleOpenNI curContext, int userId) {
  //println("onVisibleUser - userId: " + userId);
}

void keyPressed() {
  switch(key) {
  case ' ':
    context.setMirror(!context.mirror());
    break;
  }
}

//Shoulder YZ Plane Angle
float shoulderAngle(int userId) {
  PVector elbowVec = new PVector();
  context.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_RIGHT_ELBOW, elbowVec);
  PVector shoulderVec = new PVector();
  context.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_RIGHT_SHOULDER, shoulderVec);
  int[] elbow = {(int) elbowVec.x, (int) elbowVec.y, (int) elbowVec.z};
  int[] shoulder = {(int) shoulderVec.x, (int) shoulderVec.y, (int) shoulderVec.z};
  int rightArmY = shoulder[1] - elbow[1];
  int rightArmZ = shoulder[2] - elbow[2];
  float rightShoulderAngle = atan2(rightArmZ, rightArmY);
  //println("Shoulder " + rightShoulderAngle);
  return rightShoulderAngle;
}

//Shoulder XY Plane Angle
float shoulderFlapAngle(int userId) {
  PVector elbowVec = new PVector();
  context.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_RIGHT_ELBOW, elbowVec);
  PVector shoulderVec = new PVector();
  context.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_RIGHT_SHOULDER, shoulderVec);
  int[] elbow = {(int)elbowVec.x, (int)elbowVec.y, (int)elbowVec.z};
  int[] shoulder = {(int)shoulderVec.x, (int)shoulderVec.y, (int)shoulderVec.z};
  int rightArmY = shoulder[1] - elbow[1];
  int rightArmX = shoulder[0] - elbow[0];
  float rightShoulderAngle = atan2(rightArmX, rightArmY);
  //println("Shoulder Flap " + rightShoulderAngle);
  return rightShoulderAngle + (3.14/2);
}

float flapDotProduct(int userId) {
  PVector elbowVec = new PVector();
  context.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_RIGHT_ELBOW, elbowVec);
  PVector shoulderVec = new PVector();
  context.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_RIGHT_SHOULDER, shoulderVec);
  PVector hipVec = new PVector();
  context.getJointPositionSkeleton(userId,  SimpleOpenNI.SKEL_RIGHT_HIP, hipVec);
  float[] elbow = {elbowVec.x, elbowVec.y, elbowVec.z};
  float[] shoulder = {shoulderVec.x, shoulderVec.y, shoulderVec.z};
  float[] hip = {hipVec.x, hipVec.y, hipVec.z};
  float[] rightArm = {shoulder[0] - elbow[0], shoulder[1] - elbow[1]};
  float[] side = {shoulder[0] - hip[0], shoulder[1] - hip[1]};
  float armMag = sqrt(((int)rightArm[0])^2 + ((int)rightArm[1])^2);
  float sideMag = sqrt(((int)side[0])^2 + ((int)side[1])^2);
  rightArm[0] = rightArm[0]/armMag;
  rightArm[1] = rightArm[1]/armMag;
  side[0] = side[0]/sideMag;
  side[1] = side[1]/sideMag;
  float dot = (rightArm[0] * side[0]) + (rightArm[1] * side[1]);
  float angle = acos(dot);
  println(dot);
  return dot;
 
}
//Elbow YZ Plane
float elbowAngle(int userId) {
  PVector handVec = new PVector();
  context.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_RIGHT_HAND, handVec);
  PVector elbowVec = new PVector();
  context.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_RIGHT_ELBOW, elbowVec);
  int[] elbow = {(int)elbowVec.x, (int)elbowVec.y, (int)elbowVec.z};
  int[] hand = {(int)handVec.x, (int)handVec.y, (int)handVec.z};
  int rightArmY = elbow[1] - hand[1];
  int rightArmZ = elbow[2] - hand[2];
  float rightElbowAngle = atan2(rightArmZ, rightArmY);
  //println("Elbow " + rightElbowAngle);
  return rightElbowAngle;
}
