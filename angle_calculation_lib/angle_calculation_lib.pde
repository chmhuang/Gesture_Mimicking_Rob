void setup() {
  println("hi");
  int[] shoulder = {0, 800, 1000};
  int[] elbow = {200, 600, 1000};
  int[] hand = {100, 100, 1000};
  shoulderAngle(elbow, shoulder);
  shoulderFlapAngle(elbow, shoulder);
  elbowAngle(hand, elbow);
}

//Shoulder YZ Plane Angle
void shoulderAngle(int[] elbow, int[] shoulder) {
  int rightArmY = shoulder[1] - elbow[1];
  int rightArmZ = shoulder[2] - elbow[2];
  print(rightArmZ);
  print(" ");
  println(rightArmY);
  float rightShoulderAngle = atan2(rightArmZ, rightArmY);
  println("Shoulder " + rightShoulderAngle);
}

//Shoulder XY Plane Angle
void shoulderFlapAngle(int[] elbow, int[] shoulder) {
  int rightArmY = shoulder[1] - elbow[1];
  int rightArmX = elbow[0] - shoulder[0];
  float rightShoulderAngle = atan2(rightArmX, rightArmY);
  println("Shoulder Flap " + rightShoulderAngle);
}

//Elbow YZ Plane
void elbowAngle(int[] hand, int[] elbow) {
  int rightArmY = elbow[1] - hand[1];
  int rightArmZ = elbow[2] - hand[2];
  float rightElbowAngle = atan2(rightArmZ, rightArmY);
  println("Elbow " + rightElbowAngle);
}


