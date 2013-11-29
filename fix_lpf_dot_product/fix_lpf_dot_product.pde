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
int[] skeletonIndexes;
PVector[] filteredSkeleton;
final float lowPassAlpha = .99;
final float lowPassBeta = 1 - lowPassAlpha;
  
/********** Arduino **********/
// Assumed to be the first Serial port in Serial.list()
boolean isArduinoConnected;
Serial arduinoSerialPort;

PVector com = new PVector();                                   
PVector com2d = new PVector();                                   

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
  
    // set up filteredSkeleton
  skeletonIndexes = new int[] {
    SimpleOpenNI.SKEL_HEAD,
    SimpleOpenNI.SKEL_LEFT_ELBOW,
    SimpleOpenNI.SKEL_LEFT_FINGERTIP,
    SimpleOpenNI.SKEL_LEFT_FOOT,
    SimpleOpenNI.SKEL_LEFT_HAND,
    SimpleOpenNI.SKEL_LEFT_HIP,
    SimpleOpenNI.SKEL_LEFT_KNEE,
    SimpleOpenNI.SKEL_LEFT_SHOULDER,
    SimpleOpenNI.SKEL_NECK,
    SimpleOpenNI.SKEL_RIGHT_ELBOW,
    SimpleOpenNI.SKEL_RIGHT_FINGERTIP,
    SimpleOpenNI.SKEL_RIGHT_FOOT,
    SimpleOpenNI.SKEL_RIGHT_HAND,
    SimpleOpenNI.SKEL_RIGHT_HIP,
    SimpleOpenNI.SKEL_RIGHT_KNEE,
    SimpleOpenNI.SKEL_RIGHT_SHOULDER,
    SimpleOpenNI.SKEL_TORSO
  };
  int maxSkeletonIndex = 0;
  for (int i = 0; i < skeletonIndexes.length; i++) {
    maxSkeletonIndex = max(maxSkeletonIndex, skeletonIndexes[i]);
  }
  filteredSkeleton = new PVector[maxSkeletonIndex + 1];
  for (int i = 0; i < skeletonIndexes.length; i++) {
    filteredSkeleton[skeletonIndexes[i]] = new PVector();
  }

  
  // Set up Drawing Window
  size(640, 480);
  background(200, 0, 0);
  stroke(0, 0, 255);
  strokeWeight(3);
  smooth();
}
  
/********** draw() **********/
void draw() {
  // Draw Kinect Data
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

  // Perform update if possible
  if (!(userList.length > 0)) {
    return;
  }
  if (!context.isTrackingSkeleton(userList[0])) {
    return;
  }
  update();
}

/********** update() **********/
// Check (userList.length > 0 && context.isTrackingSkeleton(userList[0])) before calling 
void update() {
  updateFilteredData();

  float shoulderAng = shoulderFromDotProduct(filteredSkeleton[SimpleOpenNI.SKEL_RIGHT_ELBOW], filteredSkeleton[SimpleOpenNI.SKEL_RIGHT_SHOULDER], filteredSkeleton[SimpleOpenNI.SKEL_RIGHT_HAND]);
  float shoulderFlapAng = shoulderFlapFromDotProduct(filteredSkeleton[SimpleOpenNI.SKEL_RIGHT_ELBOW], filteredSkeleton[SimpleOpenNI.SKEL_RIGHT_SHOULDER]);
  float elbowAng = elbowFromDotProduct(filteredSkeleton[SimpleOpenNI.SKEL_RIGHT_ELBOW], filteredSkeleton[SimpleOpenNI.SKEL_RIGHT_SHOULDER], filteredSkeleton[SimpleOpenNI.SKEL_RIGHT_HAND]);

  println(toAx12Old(shoulderAng));
  println(toAx12Old(shoulderFlapAng));
  println(toAx12Old(elbowAng));
  println("\n");

  if (isArduinoConnected) {
    arduinoSerialPort.write(toAx12Old(shoulderAng));
    arduinoSerialPort.write(toAx12Old(shoulderFlapAng));
    arduinoSerialPort.write(toAx12Old(elbowAng));
    arduinoSerialPort.write('\n');
  }
}

/********** updateFilteredData() **********/
void updateFilteredData() {
  int[] userList = context.getUsers();
  int userId = userList[0];
  
  for (int i = 0; i < skeletonIndexes.length; i++) {
    PVector joint = new PVector();
    int jointIndex = skeletonIndexes[i];
    context.getJointPositionSkeleton(userId, jointIndex, joint);
    filteredSkeleton[jointIndex] = new PVector(
      lowPassAlpha * filteredSkeleton[jointIndex].x + lowPassBeta * joint.x,
      lowPassAlpha * filteredSkeleton[jointIndex].y + lowPassBeta * joint.y,
      lowPassAlpha * filteredSkeleton[jointIndex].z + lowPassBeta * joint.z
    );
  }
}

/** Helper to convert angle into Ax12 input. */
int toAx12Old(float inp) {
  return ((int) (200 + inp / (3.14/2) * 300) / 10);
}

/* Helper to calculate angle between 2 vectors. */
float angle(PVector a, PVector b, PVector c) {
  float angle01 = atan2(a.y - b.y, a.x - b.x);
  float angle02 = atan2(b.y - c.y, b.x - c.x);
  float ang = angle02 - angle01;
  return ang;
}

float shoulderFromDotProduct(PVector elbow, PVector shoulder, PVector hand) {
  PVector arm = new PVector(0, elbow.y-shoulder.y, elbow.z-shoulder.z);
  PVector side = new PVector(0, 0, 1);
  // find projection
  float dp = arm.dot(side);
  float ang = acos(dp / (arm.mag()*side.mag()));
  //println("shoulderAng = " + ang);
  
  println("magnitude of bicep vector" + arm.mag());
  float arm_mag = arm.mag();
 if (arm_mag < thresh) {
   arm = new PVector(0, hand.y-shoulder.y, hand.z-shoulder.z);
   float arm_hand_mag = arm.mag();
   if (arm_hand_mag < thresh) {
     return (3.14/2);
   } else {
     ang = vec2angle(arm, side);
     return ang;
   }
 } else {
  return ang;
 }
}

float thresh = 150;
float vec2angle(PVector v, PVector ref) {
  float dp = v.dot(ref);
  float ang = acos(dp/(v.mag()*ref.mag()));
  return ang;
}

float shoulderFlapFromDotProduct(PVector elbow, PVector shoulder) {
  PVector arm = new PVector(elbow.x-shoulder.x, elbow.y-shoulder.y, 0);
  PVector ref = new PVector(0, 1, 0);
  float ang = vec2angle(arm, ref);
  //println("shoulder flap ang = " + ang);
  float arm_mag = arm.mag();
  if (arm_mag < thresh) {
    return 3.14;
  } else {
  return ang;
  }
}

float elbowFromDotProduct(PVector elbow, PVector shoulder, PVector hand) {
  PVector forarm = new PVector(hand.x - elbow.x, hand.y - elbow.y, hand.z - elbow.z);
  PVector bicep = new PVector(shoulder.x - elbow.x, shoulder.y - elbow.y, shoulder.z - elbow.z);
  float ang = vec2angle(forarm, bicep);
  
  return (3.14-ang);
}

/********** drawSkeleton() **********/
// draw the skeleton with the selected joints
void drawSkeleton(int userId) {
  // Angle Calculation
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

/********** keyPressed() **********/
void keyPressed() {
  switch(key) {
  case ' ':
    context.setMirror(!context.mirror());
    break;
  }
}

/********** SimpleOpenNI events **********/

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

