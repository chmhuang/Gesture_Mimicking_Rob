/* --------------------------------------------------------------------------
 * Kinect to Bioloid
 * --------------------------------------------------------------------------
 * http://code.google.com/p/simple-openni
 * --------------------------------------------------------------------------
 */

import SimpleOpenNI.*;
import processing.serial.*;
import fingertracker.*;

/********** Drawing Window **********/
final color[] userClr = new color[] { 
  color(255, 0, 0), 
  color(0, 200, 200), 
  color(0, 0, 255), 
  color(255, 255, 0), 
  color(255, 0, 255), 
  color(0, 255, 255)
};
/**** Center of Mass ********/
PVector com = new PVector();
PVector com2d = new PVector();

/********** Kinect **********/
SimpleOpenNI  context;
int trackedUserIndex = 0;
int[] skeletonIndexes;
PVector[] filteredSkeleton;
final float lowPassAlpha = .9;
final float lowPassBeta = 1 - lowPassAlpha;
final float vectorMagnitudeThreshold = 150;

/********** Arduino **********/
// Assumed to be the first Serial port in Serial.list()
boolean isArduinoConnected;
Serial arduinoSerialPort;

/******* FingerTracker *******/
FingerTracker rFingers;
FingerTracker lFingers;
// default threshold about 625 mm from kinect
// should update threshold with wrist z
int rThreshold = 625;
int lThreshold = 625;
int rNumFingers = 0;
int lNumFingers = 0;
float filteredRNumFingers = 0;
float filteredLNumFingers = 0;
/********** setup() **********/
void setup() {
  println(PVector.angleBetween(new PVector(1, 1, 0), new PVector(-2, 0, 0)));
  println(PVector.angleBetween(new PVector(-2, 0, 0), new PVector(2, 2, 0)));
  // Set up Arduino
  isArduinoConnected = Serial.list().length > 0;
  if (isArduinoConnected) {
    String portName = Serial.list()[0];
    arduinoSerialPort = new Serial(this, portName, 57600);
    println("Arduino Serial Port Found: " + portName);
  } 
  else {
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

  // for Fingers
  // context.setMirror(true)
  rFingers = new FingerTracker(this, 640, 480); // need to change this size if window size changes
  rFingers.setMeltFactor(90); // melt factor is for green contour
  lFingers = new FingerTracker(this, 640, 480);
  lFingers.setMeltFactor(90);
} // end setup()

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
      stroke(255, 255, 255);
      strokeWeight(1);
      // Only draw the + for the tracked user.
      if (i == trackedUserIndex) {
        beginShape(LINES);
        vertex(com2d.x, com2d.y - 5);
        vertex(com2d.x, com2d.y + 5);
        vertex(com2d.x - 5, com2d.y);
        vertex(com2d.x + 5, com2d.y);
        endShape();
      }
      fill(0, 255, 100);
      text(Integer.toString(userList[i]), com2d.x, com2d.y);
    }
  }

  // Perform update if possible
  if (!(userList.length > trackedUserIndex)) {
    println("trackedUserIndex exceeds number of users");
    return;
  }
  if (!context.isTrackingSkeleton(userList[trackedUserIndex])) {
    println("tracked user's skeleton not ready");
    return;
  }

  // for fingers, set threshold
  int[] uList = context.getUsers();
  int uId = uList[trackedUserIndex];
  PVector rFinTipVector = new PVector(); // for setting finger tracker threshold
  PVector rHandVector = new PVector();
  int jointIndex = skeletonIndexes[10]; // should be hard-coded to fingertip
  int jointIndexHand = skeletonIndexes[12]; // for hand, hand is for prcessing background points later
  context.getJointPositionSkeleton(uId, jointIndex, rFinTipVector);
  context.getJointPositionSkeleton(uId, jointIndexHand, rHandVector);  
  rThreshold = (int) rFinTipVector.z;
  rFingers.setThreshold(rThreshold);
  int[] depthMap = context.depthMap();
  rFingers.update(depthMap);
  stroke(0, 255, 0);
  
  // for left fingers 
  uList = context.getUsers();
  uId = uList[trackedUserIndex];
  PVector lFinTipVector = new PVector(); // for setting finger tracker threshold
  PVector lHandVector = new PVector();
  jointIndex = skeletonIndexes[2]; // should be hard-coded to fingertip
  jointIndexHand = skeletonIndexes[4]; // for hand, hand is for prcessing background points later
  context.getJointPositionSkeleton(uId, jointIndex, lFinTipVector);
  context.getJointPositionSkeleton(uId, jointIndexHand, lHandVector);  
  lThreshold = (int) lFinTipVector.z;
  lFingers.setThreshold(lThreshold);
  depthMap = context.depthMap();
  lFingers.update(depthMap);
  stroke(0, 255, 0);
  
  // iterate over all the fingers found
  // and draw them as a red circles
  // Finger coordinate x and y is relative to screen, not kinect. also needs to do position -5
  noStroke();
  fill(255, 0, 0);
  // for debugging prints
  int x = 400;
  int y = 100;
  // threshold 
  int handThreshold = 100; // how far the finger points have to be away from hand
  rNumFingers = 0;
  int rCount = 0;
  for (int i = 0; i < rFingers.getNumFingers(); i++) {
    PVector position = rFingers.getFinger(i);
    PVector fingerI = new PVector(position.x-5, position.y-5);
    PVector rHandProjective = new PVector();
    context.convertRealWorldToProjective(rHandVector, rHandProjective);
    textSize(30);
    int diff = abs((int)(rHandProjective.x - fingerI.x)) + abs((int)(rHandProjective.y - fingerI.y)); // calculate diff manually instead of using dist() method
    //text("Diff " + Integer.toString(diff), x, y);
    //text(Integer.toString((int)rh2d.x) + ", " + Integer.toString((int)rh2d.y), x, y);
    //text(Integer.toString((int)fingerI.x) + ", " + Integer.toString((int)fingerI.y), x, y);
    //text(Integer.toString((int)fingerI.dist(rh2d)), x, y);
    y+=60;
    if (diff < handThreshold) {
      rCount++;
      ellipse(position.x - 5, position.y -5, 10, 10);
    }
  }
  lNumFingers = 0;
  int lCount = 0;
  for (int i = 0; i < lFingers.getNumFingers(); i++) {
    PVector position = lFingers.getFinger(i);
    PVector fingerI = new PVector(position.x-5, position.y-5);
    PVector lHandProjective = new PVector();
    context.convertRealWorldToProjective(lHandVector, lHandProjective);
    int diff = abs((int)(lHandProjective.x - fingerI.x)) + abs((int)(lHandProjective.y - fingerI.y));
    if (diff < handThreshold) {
      lCount++;
      ellipse(position.x-5, position.y-5, 10, 10);
    }
  }
  rNumFingers = rCount;
  filteredRNumFingers = lowPassAlpha * filteredRNumFingers + lowPassBeta * rNumFingers;
  
  lNumFingers = lCount;
  filteredLNumFingers = lowPassAlpha * filteredLNumFingers + lowPassBeta * lNumFingers;
  text("rNumFingers " + Integer.toString(rNumFingers), 400, 100);
  text("lNumFingers" + Integer.toString(lNumFingers), 400, 200);
  
  
  update();
} // end draw()

/********** update() **********/
// Check (userList.length > 0 && context.isTrackingSkeleton(userList[trackedUserIndex])) before calling 
void update() {
  updateFilteredData();
  byte[] packet = generatePacket();

  // Print Packet
  print("Packet Data: ");
  for (int i = 0; i < packet.length - 1; i++) {
    print((int) packet[i]);
    print(" ");
  }
  print('\n');

  // Draw Packet
  textSize(40);
  int x = 10;
  int y = 50;
  for (int i = 0; i < packet.length - 1; i++) {
    text(Integer.toString((int) packet[i]), x, y);
    y += 50;
  }
  y += 50;
  text("rThreshold = " + rThreshold, x, y);


  // Send Packet
  if (isArduinoConnected) {
    arduinoSerialPort.write(packet);
  }
}

/********** updateFilteredData() **********/
void updateFilteredData() {
  int[] userList = context.getUsers();
  int userId = userList[trackedUserIndex];

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

/********** generatePacket() **********/
byte[] generatePacket() {
  byte[] packet = new byte[6];

  int i = 0;
  packet[i++] = toAx12(shoulderRotationAngle(true), false);
  packet[i++] = toAx12(shoulderFlapAngle(true), false);
  packet[i++] = toAx12(angle3(true), true);
  packet[i++] = toAx12(elbowBendAngle(true), true);
  packet[i++] = fingerGrab(true);

  packet[i++] = '\n'; // end of packet

  return packet;
}
/********** toAx12() **********/
// Maps angle (0 to PI) to Ax12 input (20 to 80)
// if inverted is true, maps 0 to 80, and PI to 20
byte toAx12(float angle, boolean inverted) {
  if (inverted) { 
    if (angle > Math.PI) {
      angle = (float)Math.PI;
    }
    return (byte) (80 - 60 * angle / Math.PI);
  } 
  else {
     if (angle > Math.PI) {
      angle = (float) Math.PI;
    }
    return (byte) (20 + 60 * angle / Math.PI);
  }
}

/********** project() **********/
// If subtractFromV1 is false: returns the projection of v1 onto v2.
// If subtractFromV1 is true: returns v1 - projection. This can be used to project onto a plane, where v2 is the normal of the plane.
PVector project(PVector v1, PVector v2, boolean subtractFromV1) {
  PVector projection = PVector.mult(v2, v1.dot(v2) / v2.mag() / v2.mag());
  if (subtractFromV1) {
    return PVector.sub(v1, projection);
  } 
  else {
    return projection;
  }
}

/********** shoulderRotationAngle() **********/
// Angle between (0,0,1) and the proejction of the arm (shoulder, elbow/hand joints) onto the YZ plane.
// Returns angle 0 to PI.
float shoulderRotationAngle(boolean rightSide) {
  PVector elbow = rightSide ? filteredSkeleton[SimpleOpenNI.SKEL_RIGHT_ELBOW] : filteredSkeleton[SimpleOpenNI.SKEL_LEFT_ELBOW];
  PVector shoulder = rightSide ? filteredSkeleton[SimpleOpenNI.SKEL_RIGHT_SHOULDER] : filteredSkeleton[SimpleOpenNI.SKEL_LEFT_SHOULDER];
  PVector hand = rightSide ? filteredSkeleton[SimpleOpenNI.SKEL_RIGHT_HAND] : filteredSkeleton[SimpleOpenNI.SKEL_LEFT_HAND];

  PVector armYZ = project(PVector.sub(elbow, shoulder), new PVector(1, 0, 0), true);
  PVector rotationAngle0 = new PVector(0, 0, 1);

  //  println("magnitude of bicep vector" + arm.mag());

  if (armYZ.mag() < vectorMagnitudeThreshold) {
    // If armYZ magnitude using elbow is too small, try using hand
    armYZ = project(PVector.sub(hand, shoulder), new PVector(1, 0, 0), true);

    if (armYZ.mag() < vectorMagnitudeThreshold) {
      return (float) Math.PI / 2;
    } 
    else {
      return PVector.angleBetween(armYZ, rotationAngle0);
    }
  } 
  else {
    return PVector.angleBetween(armYZ, rotationAngle0);
  }
}

/********** shoulderFlapAngle() **********/
// Angle between (0,1,0) and the projection of the arm (shoulder, elbow joints) onto the XY plane.
// Returns angle 0 to PI.
// reference depends on motor 1 angle
float shoulderFlapAngle(boolean rightSide) {
  PVector elbow = rightSide ? filteredSkeleton[SimpleOpenNI.SKEL_RIGHT_ELBOW] : filteredSkeleton[SimpleOpenNI.SKEL_LEFT_ELBOW];
  PVector shoulder = rightSide ? filteredSkeleton[SimpleOpenNI.SKEL_RIGHT_SHOULDER] : filteredSkeleton[SimpleOpenNI.SKEL_LEFT_SHOULDER];

  PVector arm = PVector.sub(elbow, shoulder);
  float refAngle = shoulderRotationAngle(rightSide);
  PVector flapAngle0 = new PVector(0, sin(refAngle), -1 * cos(refAngle));

  return PVector.angleBetween(arm, flapAngle0);
  /*
  if (armXY.mag() < vectorMagnitudeThreshold) {
   return (float) Math.PI;
   } else {
   return PVector.angleBetween(armXY, flapAngle0);
   }*/
}

/********** elbowBendAngle() **********/
// Angle between shoulder, elbow, and hand joints.
// Returns angle 0 to PI.
float elbowBendAngle(boolean rightSide) {
  PVector elbow = rightSide ? filteredSkeleton[SimpleOpenNI.SKEL_RIGHT_ELBOW] : filteredSkeleton[SimpleOpenNI.SKEL_LEFT_ELBOW];
  PVector shoulder = rightSide ? filteredSkeleton[SimpleOpenNI.SKEL_RIGHT_SHOULDER] : filteredSkeleton[SimpleOpenNI.SKEL_LEFT_SHOULDER];
  PVector hand = rightSide ? filteredSkeleton[SimpleOpenNI.SKEL_RIGHT_HAND] : filteredSkeleton[SimpleOpenNI.SKEL_LEFT_HAND];

  PVector forarm = PVector.sub(hand, elbow);
  PVector bicep = PVector.sub(shoulder, elbow);
  return PVector.angleBetween(forarm, bicep);
} // end elbowBendAngle()

/********** fingerGrab() *********/
byte fingerGrab(boolean rightSide) {
  // see how many fingers currently and compared to previous value
  // needs to store old values because moving opened hand around will induce a lot of noise
  int numFingers = rightSide ? rNumFingers : lNumFingers; 
  if (numFingers > 2.5) {
    return (byte) 80;
  } 
  else if (numFingers <= 2.5) {
    return (byte) 50;
  }
  return (byte) 50; // default to close hand
} // end fingerGrab()

/*********** angle3 ***************/
float angle3(boolean rightSide) {
  // a is forarm vector
  // b is bicep vector/upper arm vector
  PVector a = new PVector();
  PVector b = new PVector();
  PVector c = new PVector();
  PVector elbow = rightSide ? filteredSkeleton[SimpleOpenNI.SKEL_RIGHT_ELBOW] : filteredSkeleton[SimpleOpenNI.SKEL_LEFT_ELBOW];
  PVector shoulder = rightSide ? filteredSkeleton[SimpleOpenNI.SKEL_RIGHT_SHOULDER] : filteredSkeleton[SimpleOpenNI.SKEL_LEFT_SHOULDER];
  PVector hand = rightSide ? filteredSkeleton[SimpleOpenNI.SKEL_RIGHT_HAND] : filteredSkeleton[SimpleOpenNI.SKEL_LEFT_HAND];

  PVector forarm = PVector.sub(hand, elbow);
  PVector bicep = PVector.sub(shoulder, elbow);
  a = forarm;
  b = bicep;
  
  float n = a.dot(b) / b.dot(b);
  PVector rSide = PVector.mult(b, n);
  c = PVector.sub(a, rSide);
  float angle1 = shoulderRotationAngle(true);
  PVector p = new PVector(0, -sin(angle1), cos(angle1));
  return ((acos((p.dot(c)) / (p.mag() * c.mag()))) * -1) + ((float) Math.PI/2); // for some reason formula is inverted that's why we need to *-1 + pi/2
  
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
  case 's':
    int numUsers = context.getUsers().length;
    trackedUserIndex = (trackedUserIndex + 1) % numUsers;
    print("Switching user: new index is ");
    println(trackedUserIndex);
    delay(1000);
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
  int numUsers = context.getUsers().length;
  if (trackedUserIndex >= numUsers) {
    trackedUserIndex--;
    print("Switching user: new index is ");
    println(trackedUserIndex);
  }
}

void onVisibleUser(SimpleOpenNI curContext, int userId) {
  //println("onVisibleUser - userId: " + userId);
}

