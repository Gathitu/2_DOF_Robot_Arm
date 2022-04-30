#include <Servo.h>
#include <Math.h>

const float pi = 3.14159265359; 
enum AllProcesses {Init, DrillStartPointEntry, RobotArmMotion, DrillEndPointEntry,  DrillOperation, ReturnToStartPosition};
enum AllProcesses currentProcess = Init;
bool robotArmIsMoving = false; ///to not accept input into COM when robot arm is moving
///the next 2 booleans prevent running of servo code if the required servo isnt connected to Arduino
bool canMove1stMotor = false;
bool canMove2ndMotor = false;

const unsigned int servo_motor1_constraints[2] = {0, 180};
const unsigned int servo_motor2_constraints[2] = {0, 180};
unsigned int servo_motor1_position= 90;
unsigned int servo_motor2_position= 90;

Servo servo_motor1;
Servo servo_motor2;

long homePosition[3] = {6.2, 5.2, 0.0}; ///axis positions can have -ve values
long drillStartPosition[3];
long drillEndPosition[3];
unsigned long link1_length = 6.2; ///in cm
unsigned long link2_length = 5.2; ///in cm

void setup() {

  Serial.begin(9600);
  initProcess();

  if(canMove1stMotor) servo_motor1.attach(3);
  if(canMove2ndMotor) servo_motor2.attach(4);

  if(canMove1stMotor) servo_motor1.write(servo_motor1_position);
  if(canMove2ndMotor) servo_motor2.write(servo_motor2_position);
}

void initProcess(){
  Serial.println(displayString());
  advanceCurrentProcess();
  Serial.println(displayString());
}

void loop() {
  if (Serial.available() > 0 && !robotArmIsMoving) { ///If input is available and robot arm is not moving
    String incomingString = Serial.readString();
    checkValidityOfInput(incomingString);
  }

}

String displayString() {
  String dspString  = "\nHome Position -> " + calculateEndEffectorPosition(servo_motor1_position, servo_motor2_position);

  if (currentProcess == Init) { } 
  else if (currentProcess == DrillStartPointEntry) {
      dspString  = "\nEnter Start Point for Drill Operation. Format example: " + calculateEndEffectorPosition(servo_motor1_position, servo_motor2_position);
  } else if (currentProcess == DrillEndPointEntry) {
      dspString = "\nEnter End Point for Drill Operation";
  } else if (currentProcess == RobotArmMotion) {
      dspString = "\nMoving Robotic Arm ...";
  } else if (currentProcess == DrillOperation) {
      dspString = "\nDrilling in progress ...";
  } else if (currentProcess == ReturnToStartPosition) {
      dspString = "\nReturning to Start Position ...";
  }
  return dspString;
}

void advanceCurrentProcess() {
  if (currentProcess == Init) {
    currentProcess =  DrillStartPointEntry;
  } 
  else if (currentProcess == DrillStartPointEntry) {
    currentProcess = RobotArmMotion;
  } 
  else if (currentProcess == RobotArmMotion) {
    currentProcess = DrillEndPointEntry;
  }
  else if (currentProcess == DrillEndPointEntry) {
    currentProcess = DrillOperation;
  }
  else if (currentProcess == DrillOperation) {
    currentProcess = ReturnToStartPosition;
  }
  else if (currentProcess == ReturnToStartPosition){
    currentProcess = DrillStartPointEntry;
  }
}

void fallBackFromCurrentProcess() {
  if (currentProcess == Init) {
    currentProcess =  DrillStartPointEntry;
  } 
  else if (currentProcess == DrillStartPointEntry) {
    currentProcess = Init;
  } 
  else if (currentProcess == RobotArmMotion) {
    currentProcess = DrillStartPointEntry;
  }
  else if (currentProcess == DrillEndPointEntry) {
    currentProcess = RobotArmMotion;
  }
  else if (currentProcess == DrillOperation) {
    currentProcess = DrillEndPointEntry;
  }
  else if (currentProcess == ReturnToStartPosition){
    currentProcess = DrillOperation;
  }
}

void checkValidityOfInput(String input) {
   if (currentProcess == ReturnToStartPosition) {///Start over process by typing anything in the terminal
     Serial.println(displayString());
     calculateDisplacementsInAllAxes();
     return; 
    }
  if(currentProcess != DrillStartPointEntry && currentProcess != DrillEndPointEntry) return;
  
  const unsigned int inputLength = input.length() -1; ///NOTE: length is added by 1
  String firstCharacter = input.substring(0,1);
  String lastCharacter = input.substring(inputLength - 1);
  
  if(firstCharacter!= "[" && lastCharacter!= "]"){
    Serial.println("Doesnt have brackets");
    displayInvalidInput(input);
    return;
  }

  char separator = ',';

  String splits[3];
  String split = "";
  unsigned int arrayLength = 0;
  for(int i=0; i< inputLength; i++){
    if(input[i] == '[' || input[i] == ' '){}///do nothing
    else if(input[i] == separator || input[i] == ']'){
      if(arrayLength <3) splits[arrayLength] = split;
      arrayLength +=1;
      split = "";
    }
    else{
      split += input[i];
    }
  }

  String concatenatedInputMatrix = "[" + String(splits[0]) + "," +
    String(splits[1]) + "," + String(splits[2]) + "]";

  if(currentProcess == DrillStartPointEntry){
    Serial.println("Start Position: " + concatenatedInputMatrix);
  }else if(currentProcess == DrillEndPointEntry){
    Serial.println("End Position: " + concatenatedInputMatrix);
  }

  if(arrayLength != 3){///3 elements
    Serial.println("Array length shld be 3");
    displayInvalidInput(input);
    return;
  }

  ///String.toInt() returns a long(from -2.148 billion to 2.148 billion bytes) 
  ///or a 0 if input starts with a non-integer value or if input is 0
  if (currentProcess == DrillStartPointEntry) {
    drillStartPosition[0] = String(splits[0]).toInt();
    drillStartPosition[1] = String(splits[1]).toInt();
    drillStartPosition[2] = String(splits[2]).toInt();
  }else if(currentProcess == DrillEndPointEntry){
    drillEndPosition[0] = String(splits[0]).toInt();
    drillEndPosition[1] = String(splits[1]).toInt();
    drillEndPosition[2] = String(splits[2]).toInt();
  }
  
  calculateDisplacementsInAllAxes();

}
void displayInvalidInput(String input){
  Serial.println("INVALID INPUT: " + input);
  Serial.println(displayString());
}

void calculateDisplacementsInAllAxes(){
  if (currentProcess != ReturnToStartPosition){
    advanceCurrentProcess();
    Serial.println(displayString());  
  }
  
  bool newPositionIsOutOfRange = false;
  robotArmIsMoving = true;
  if(currentProcess == RobotArmMotion){ ///from  homePosition to drillStartPosition
    newPositionIsOutOfRange = !calculateAngleDisplacement(drillStartPosition); 
  }
  else if(currentProcess == DrillOperation){ ///from  drillStartPosition to drillEndPosition
    newPositionIsOutOfRange = !calculateAngleDisplacement(drillEndPosition); 
  }
  else{///currentProcess = ReturnToStartPosition ,, ///from  drillEndPosition to drillStartPosition
    newPositionIsOutOfRange = !calculateAngleDisplacement(drillStartPosition); 
  }
    
  robotArmIsMoving = false;
  if(newPositionIsOutOfRange) {
    fallBackFromCurrentProcess();
    Serial.println(displayString()); 
    return;
  }
  
  if(currentProcess == DrillOperation){
    Serial.println("\nDrill Operation Completed. Click Enter to return to Start Position");
    advanceCurrentProcess();
  }else if(currentProcess == ReturnToStartPosition){ ///Start over process
    Serial.println("\n");
    advanceCurrentProcess();
    Serial.println(displayString());
  }else{
    advanceCurrentProcess();
    Serial.println(displayString());
  }
}

bool calculateAngleDisplacement (long newPosition[3]) {///if new position is out of range, return false
  long new_x_position = newPosition[0];
  long new_y_position = newPosition[1];
  long new_z_position = newPosition[2];

  double angle1InRadian = atan(new_y_position/new_x_position);
  double angle1 = angle1InRadian*(180/pi); // convert the angle from radian to degree 
  double angle2 = 0.0;
  double angle2InRadian;
  double m;///ensures precise answer bcoz arithmetic operation between a long and a double will round off answer impacting outcome
  
  if(new_z_position < link1_length){
    m = link1_length-new_z_position; 
    angle2InRadian = acos( m/link2_length );
    angle2 = angle2InRadian*(180/pi);
  }
  else if(new_z_position > link1_length){
    m = new_z_position-link1_length; 
    angle2InRadian = asin( m/ link2_length );
    angle2 = 90.0 + (angle2InRadian*(180/pi));
  }
  else{//new_z_position == link1_length
    angle2 = 90.0;
  }

  if(isnan(angle1) || isnan(angle2)||
    angle1 < servo_motor1_constraints[0] || angle1 > servo_motor1_constraints[1] || 
    angle2 < servo_motor2_constraints[0] || angle2 > servo_motor2_constraints[1]){
      Serial.println("\nNew Position is out of range!\n");
      return false;
  }
  
  executeInput(angle1,angle2);
  return true;
}

String calculateEndEffectorPosition(unsigned int angle1, unsigned int angle2){///calculates position in terms of [X,Y,Z]
  String str = "[";
  double x_position = 0.0;
  double y_position = 0.0;
  double z_position = 0.0;
  double angle1InRadian = angle1*(pi/180);
  double angle2InRadian = angle1*(pi/180);

  x_position = link2_length * cos(angle1InRadian);
  y_position = link2_length * sin(angle1InRadian);

  if(angle2 < 90){
     z_position = link1_length - (link2_length * cos(angle2InRadian));
  }if(angle2 > 90){
     z_position = link1_length + (link2_length * sin(angle2InRadian));
  }else{//angle2 == 90
    z_position = link1_length;
  }

  str += String(x_position) + "," + String(y_position) + "," + String(z_position) + "]";
  return str;
}


void executeInput(double angleForServoMotor1,double angleForServoMotor2){
  double angle1 = angleForServoMotor1;
  double angle2 = angleForServoMotor2;
  if(angle1 < servo_motor1_constraints[0]) angle1 = servo_motor1_constraints[0];
    else if(angle1 > servo_motor1_constraints[1]) angle1 = servo_motor1_constraints[1];
  if(angle2 < servo_motor2_constraints[0]) angle2 = servo_motor2_constraints[0];
    else if(angle2 > servo_motor2_constraints[1]) angle2 = servo_motor2_constraints[1];

  double angleDifference;///angle that a servo motor will be displaced
  angleDifference = angle1 - servo_motor1_position;
  Serial.println(displacementString('1',angleDifference));
  
  angleDifference = angle2 - servo_motor2_position;
  Serial.println(displacementString('2',angleDifference));
 
  ///feed angle positions to both servo-motors at the same time but step-wise
  if (angle1>angle2) {
    for(int i=0; i<angle1; i++){
      servo_motor1_position += i;
      if(canMove1stMotor) servo_motor1.write(servo_motor1_position);
      else delay(1000);
      for(int k=0; k<angle2; k++){
        servo_motor2_position += i;
        if(canMove2ndMotor) servo_motor2.write(servo_motor2_position);
        else delay(1000);
      }
      if(currentProcess == DrillOperation) delay(800); else delay(50); ///Feed rate should be slower compared to moving links
    }
  }else{///if angle2?angle1 or angle2==angle1
    for(int k=0; k<angle2; k++){
      servo_motor2_position += k;
      if(canMove2ndMotor) servo_motor2.write(servo_motor2_position);
      else delay(1000);
      for(int i=0; i<angle1; i++){
        servo_motor1_position += i;
        if(canMove1stMotor) servo_motor1.write(servo_motor1_position);
        else delay(1000);
      }
      if(currentProcess == DrillOperation) delay(800); else delay(50);
    }
  }

}

String displacementString(char joint,double angleDifference) {
  return "Servo motor " + String(joint) + " displacement: " + String(angleDifference) + "º";
}
