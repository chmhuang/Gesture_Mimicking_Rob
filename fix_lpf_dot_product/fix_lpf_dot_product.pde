/* --------------------------------------------------------------------------
 * Shoulder Elbow to Bioloid
 * --------------------------------------------------------------------------
 * http://code.google.com/p/simple-openni
 * --------------------------------------------------------------------------
 */

import SimpleOpenNI.*;
import processing.serial.*;
Serial myPort;
SimpleOpenNI  context;
color[]       userClr = new color[] { 
  color(255, 0, 0), 
  color(0, 200, 200), 
  color(0, 0, 255), 
  color(255, 255, 0), 
  color(255, 0, 255), 
  color(0, 255, 255)
};
PVector com = new PVector();                                   
PVector com2d = new PVector();                                   

void setup() {
  size(640, 480);
  String portName = Serial.list()[0];
  myPort = new Serial(this, portName, 9600);
  context = new SimpleOpenNI(this);
  if (context.isInit() == false) {
    println("Can't init SimpleOpenNI, maybe the camera is not connected!"); 
    exit();
    return;
  }

  // enable depthMap generation 
  context.enableDepth();

  // enable skeleton generation for all joints.
  context.enableUser();
  background(200, 0, 0);
  stroke(0, 0, 255);
  strokeWeight(3);
  smooth();
}
  PVector oldRShoulder = new PVector();
  PVector oldRElbow = new PVector();
  PVector oldRHand = new PVector();
  PVector oldRHip = new PVector();
  PVector oldRKnee = new PVector();
  PVector oldRFoot = new PVector();
  PVector oldLShoulder = new PVector();
  PVector oldLElbow = new PVector();
  PVector oldLHand = new PVector();
  PVector oldLHip = new PVector();
  PVector oldLKnee = new PVector();
  PVector oldLFoot = new PVector();  
  PVector oldTorso = new PVector();
  PVector oldNeck = new PVector();
  
  PVector filteredRShoulder = new PVector();
  PVector filteredRElbow = new PVector();
  PVector filteredRHand = new PVector();
  PVector filteredRHip = new PVector();
  PVector filteredRKnee = new PVector();
  PVector filteredRFoot = new PVector();
  PVector filteredLShoulder = new PVector();
  PVector filteredLElbow = new PVector();
  PVector filteredLHand = new PVector();
  PVector filteredLHip = new PVector();
  PVector filteredLKnee = new PVector();
  PVector filteredLFoot = new PVector();  
  PVector filteredTorso = new PVector();
  PVector filteredNeck = new PVector();
  
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
      PVector rElbow = new PVector();
      PVector rShoulder = new PVector();
      PVector rHand = new PVector();
      PVector rHip = new PVector();
      PVector rKnee = new PVector();
      PVector rFoot = new PVector();
      PVector lElbow = new PVector();
      PVector lShoulder = new PVector();
      PVector lHand = new PVector();
      PVector lHip = new PVector();
      PVector lKnee = new PVector();
      PVector lFoot = new PVector();
      PVector torso = new PVector();
      PVector neck = new PVector();
      int userId = userList[0];
      context.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_RIGHT_ELBOW, rElbow);
      context.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_RIGHT_SHOULDER, rShoulder);
      context.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_RIGHT_HAND, rHand);
      context.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_RIGHT_HIP, rHip);
      context.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_RIGHT_KNEE, rKnee);
      context.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_RIGHT_FOOT, rFoot);
      context.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_LEFT_ELBOW, lElbow);
      context.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_LEFT_SHOULDER, lShoulder);
      context.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_LEFT_HAND, lHand);
      context.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_LEFT_HIP, lHip);
      context.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_LEFT_KNEE, lKnee);
      context.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_LEFT_FOOT, lFoot);
      context.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_TORSO, torso);
      context.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_NECK, neck);
      
      float alpha = 0.99;
      float beta = 1 - alpha;
      filteredRShoulder = new PVector(alpha * oldRShoulder.x + beta * rShoulder.x,
                           alpha * oldRShoulder.y + beta * rShoulder.y, 
                           alpha * oldRShoulder.z + beta * rShoulder.z);
      filteredRElbow = new PVector(alpha * oldRElbow.x + beta * rElbow.x,
                           alpha * oldRElbow.y + beta * rElbow.y, 
                           alpha * oldRElbow.z + beta * rElbow.z);
      filteredRHand = new PVector(alpha * oldRHand.x + beta * rHand.x,
                           alpha * oldRHand.y + beta * rHand.y, 
                           alpha * oldRHand.z + beta * rHand.z);
 
      oldRShoulder = rShoulder;
      oldRElbow = rElbow;
      oldRHand = rHand;
      float shoulderAng = shoulderFromDotProduct(filteredRElbow, filteredRShoulder, filteredRHand);
      float shoulderFlapAng = shoulderFlapFromDotProduct(filteredRElbow, filteredRShoulder);
      float elbowAng = elbowFromDotProduct(filteredRElbow, filteredRShoulder, filteredRHand);
     // println("shoulderAng " + shoulderAng);
     // println("shoulderFlapAng " + shoulderFlapAng);
      println(toAx12Old(shoulderAng));
      println(toAx12Old(shoulderFlapAng));
      println(toAx12Old(elbowAng));
      myPort.write(toAx12Old(shoulderAng));
      myPort.write(toAx12Old(shoulderFlapAng));
      myPort.write(toAx12Old(elbowAng));
      //println(toAx12(shoulderAng));
      //myPort.write(toAx12(filteredShoulderFlap));
    //  println(toAx12(filteredShoulderFlap));
      myPort.write('\n');
      //delay(100);
    }
  }
}
/** Helper to convert angle into Ax12 input. */
int toAx12Old(float inp) {
  return ((int) (200 + inp / (3.14/2) * 300) / 10);
}
int toAx12(float inp) {
  return ((int) (inp/3.14 * 600) + 200);
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
float shoulderAngle(PVector elbowVec, PVector shoulderVec) {
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
