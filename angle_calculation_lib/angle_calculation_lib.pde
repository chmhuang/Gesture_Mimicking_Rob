void setup() {
  println("hi");
  int[] shoulder = {100, 800, 1000};
  int[] elbow = {100, 600, 1000};
  int[] hand = {100, 100, 1000};
  shoulderAngle(elbow, shoulder);
  elbowAngle(hand, elbow);
}

//Shoulder Rotation Angle
void shoulderAngle(int[] elbow, int[] shoulder) {
  int rightArmY = shoulder[1] - elbow[1];
  int rightArmZ = shoulder[2] - elbow[2];
  print(rightArmZ);
  print(" ");
  println(rightArmY);
  float rightShoulderAngle = atan2(rightArmZ, rightArmY);
  println("Shoulder " + rightShoulderAngle);
}

void elbowAngle(int[] hand, int[] elbow) {
  int rightArmY = elbow[1] - hand[1];
  int rightArmZ = elbow[2] - hand[2];
  float rightElbowAngle = atan2(rightArmZ, rightArmY);
  println("Elbow " + rightElbowAngle);
}
