#include <Arduino.h>
#include <math.h>

// Converts degrees to radians. M_PI is defined in <math.h>.
#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)

// Converts radians to degrees.
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI)

// speed profile variables
struct Point
{
  long x;
  long y;
};
class Curve // class for storing bezier curve control points
{
public:
  Point start;
  Point control1; // control points for bezier curve
  Point control2;
  Point end;
};

// define pins for motors
#define J1stepPin 16
#define J1dirPin 20

#define J2stepPin 17
#define J2dirPin 21

#define J3stepPin 19
#define J3dirPin 23

#define J4stepPin 18
#define J4dirPin 22

#define J1enablePin 41
#define J2enablePin 40
#define J3enablePin 15
#define J4enablePin 14

#define J1calPin 24
#define J2calPin 25
#define J3calPin 26
#define J4calPin 27

// calibration rotation direction
#define J1caldir 0
#define J2caldir 0
#define J3caldir 1
#define J4caldir 0

// default rotation direction change if needed
#define J1rotdir 1
#define J2rotdir 1
#define J3rotdir 0
#define J4rotdir 1

// steps per revolution for each motor
#define J1stepsPerRev 20000.0 // microstepping 800 , ratio 25:1
#define J2stepsPerRev 20000.0 // microstepping 5000, ratio 4:1
#define J3stepsPerRev 200.0   // microstepping on driver is set to 400 but for better performance of the robot it is set in software to 200 
#define J4stepsPerRev 16666.666666666666666666666666667// microstepping 10000, ratio 1,6666666666666666666666666666667:1

#define J1stepsPerDeg 55.55555555555556
#define J2stepsPerDeg 55.55555555555556
#define J3stepsPerMM  78.74015748031496 //(previously was 157,4803149606299) 2.54mm per rev
#define J4stepsPerDeg 46.29629629629630

// robot axis limits
#define J1LimPos 125.0
#define J1LimNeg -116.0
#define J2LimPos 120.0
#define J2LimNeg -129.0
#define J3LimUpper 0.0
#define J3LimLower 170.0
#define J4LimPos 120.0
#define J4LimNeg -190.0

// steps from home position to zero position
int J1StepsDef = int(J1LimNeg * J1stepsPerDeg);
int J2StepsDef = int(J2LimNeg * J2stepsPerDeg);
int J3StepsDef = int(J3LimUpper * J3stepsPerMM);
int J4StepsDef = int(J4LimNeg * J4stepsPerDeg);

int J1MaxStepsPos = int(J1LimPos * J1stepsPerDeg);
int J2MaxStepsPos = int(J2LimPos * J2stepsPerDeg);
int J3MaxStepsPos = int(J3LimLower * J3stepsPerMM);
int J4MaxStepsPos = int(J4LimPos * J4stepsPerDeg);

String cmdbuff3;
String cmdbuff2;
String cmdbuff1;
String receivedData;
String processedData;
String instruction;

int debug = 0;
unsigned long previousMillis = 0;
const long interval = 300; // interval at which to blink (milliseconds)

// global robot position vars - absolute steps from home position
int32_t J1stepsMaster = 0;
int32_t J2stepsMaster = 0;
int32_t J3stepsMaster = 0;
int32_t J4stepsMaster = 0;

// ================== ROBOT PARAMETERS ==================
// robot kinematic parameters
float L1 = 250.0;
float L2 = 150.0;
float d1 = 217.0;
float dcor = 22.5;
int kinError = 0;
int gripperSymetry = 0;

// forward kinematics solution variables
float fw_x = 0;
float fw_y = 0;
float fw_z = 0;
float fw_fi = 0;

// inverse kiknematics variables  //J3 and J4 need to be changed
float inv_J1deg[2] = {0.0, 0.0}; // left and right solution
float inv_J2deg[2] = {0.0, 0.0};
float inv_J3distance = 0.0;
float inv_J4deg[2] = {0.0, 0.0};

int leftHandedPossible = 0;
int rightHandedPossible = 0;

// enum TeensyStates
enum TeensyStates
{
  IDLE,
  PROCESSING_INSTRUCTION,
  CLEARING_BUFFER1,
  // instructions
  DL, // delay - wait for time in ms - this function is blocking
  EN, // enable motors
  HS, // home to limit switches until all switches are triggered
  JJ, // jog joints
  MJ, // move joints
  KF, // forward kinematics test
  TL, // test limit switches
  TD, // test drive motors
  RP, // robot position

} TeensyState;

// ================== KINEMATIC FUNCTIONS ==================
void ForwardKinematicsSolver(float J1angle, float J2angle, float J3distance, float J4angle)
{
  float J1angleRad = degreesToRadians(J1angle);
  float J2angleRad = degreesToRadians(J2angle);
  float J4angleRad = degreesToRadians(J4angle);
  fw_x = L1 * cos(J1angleRad) + L2 * cos(J1angleRad + J2angleRad);
  fw_y = L1 * sin(J1angleRad) + L2 * sin(J1angleRad + J2angleRad);
  fw_z = d1 - J3distance - dcor;
  fw_fi = radiansToDegrees((J1angleRad + J2angleRad + J4angleRad));
}
void InverseKinematicsSolver(float inv_x, float inv_y, float inv_z, float inv_fi)
{
  kinError = 0; // reset the error flag
  float J4angleSolLeftRadAlt2 = 0;
  float J4angleSolLeftRadAlt3 = 0;
  float J4angleSolRightRadAlt2 = 0;
  float J4angleSolRightRadAlt3 = 0;

  // calculate the angle of the second joint
  float inv_fiRad = degreesToRadians(inv_fi);
  float acosArg = (pow(inv_x, 2) + pow(inv_y, 2) - pow(L1, 2) - pow(L2, 2)) / (2 * L1 * L2);
  if (acosArg > 1.0 || acosArg < -1.0)
  {
    kinError = 1;
    return;
  }
  float J2angleSolRightRad = acos(acosArg);
  float J2angleSolLeftRad = -J2angleSolRightRad;
  // calculate the angle of the first joint
  float J1angleSolRightRad = atan2(L2 * sin(J2angleSolLeftRad) * inv_x + (L1 + L2 * cos(J2angleSolLeftRad)) * inv_y, (L1 + L2 * cos(J2angleSolLeftRad)) * inv_x - L2 * sin(J2angleSolLeftRad) * inv_y);
  float J1angleSolLeftRad = -J1angleSolRightRad + 2 * atan2(inv_y, inv_x);
  // calculate the angle of the third joint
  float J4angleSolLeftRad = inv_fiRad - J1angleSolLeftRad - J2angleSolLeftRad;
  float J4angleSolRightRad = inv_fiRad - J1angleSolRightRad - J2angleSolRightRad;

  inv_J3distance = d1 - inv_z - dcor;

  // corrections for different quadrants
  // J1 angle left solution correction
  if (J1angleSolLeftRad > M_PI)
  {
    J1angleSolLeftRad = J1angleSolLeftRad - M_TWOPI;
  }

  if (J1angleSolLeftRad < -M_PI)
  {
    J1angleSolLeftRad = M_TWOPI + J1angleSolLeftRad;
  }
  // J1 angle right solution correction
  if (J1angleSolRightRad > M_PI)
  {
    J1angleSolRightRad = J1angleSolRightRad - M_TWOPI;
  }

  if (J1angleSolRightRad < -M_PI)
  {
    J1angleSolRightRad = J1angleSolRightRad + M_TWOPI;
  }
  // J3 angle left solution correction
  if (J4angleSolLeftRad < -M_PI)
  {
    J4angleSolLeftRad = J4angleSolLeftRad + M_TWOPI;
  }
  if (J4angleSolLeftRad > M_PI)
  {
    J4angleSolLeftRad = J4angleSolLeftRad - M_TWOPI;
  }
  // J3 angle right solution correction
  if (J4angleSolRightRad < -M_PI)
  {
    J4angleSolRightRad = J4angleSolRightRad + M_TWOPI;
  }
  if (J4angleSolRightRad > M_PI)
  {
    J4angleSolRightRad = J4angleSolRightRad - M_TWOPI;
  }
  if (J4angleSolRightRad < -M_TWOPI)
  {
    J4angleSolRightRad = J4angleSolRightRad + M_TWOPI;
  }

  // convert the angles to degrees
  inv_J1deg[0] = radiansToDegrees(J1angleSolLeftRad);
  inv_J1deg[1] = radiansToDegrees(J1angleSolRightRad);
  inv_J2deg[0] = radiansToDegrees(J2angleSolLeftRad);
  inv_J2deg[1] = radiansToDegrees(J2angleSolRightRad);
  inv_J3distance = d1 - inv_z - dcor; // entire calculation for z axis
  inv_J4deg[0] = radiansToDegrees(J4angleSolLeftRad);
  inv_J4deg[1] = radiansToDegrees(J4angleSolRightRad);
}
void checkLimitsInverse()
{
  leftHandedPossible = 0;
  rightHandedPossible = 0;
  // check if the solution is possible
  if (((J1LimNeg < inv_J1deg[0]) && (inv_J1deg[0] < J1LimPos)) && ((J2LimNeg < inv_J2deg[0]) && (inv_J2deg[0] < J2LimPos)) && ((J4LimNeg < inv_J4deg[0]) && (J4LimPos > inv_J4deg[0])))
  {
    leftHandedPossible = 1;
  }
  else
  {
    Serial.println("left handed solution is over angle limits");
  }

  if (((J1LimNeg < inv_J1deg[1]) && (inv_J1deg[1] < J1LimPos)) && ((J2LimNeg < inv_J2deg[1]) && (inv_J2deg[1] < J2LimPos)) && ((J4LimNeg < inv_J4deg[1]) && (J4LimPos > inv_J4deg[1])))
  {
    rightHandedPossible = 1;
  }
  else
  {
    Serial.println("right handed solution is over angle limits");
  }
}
int checkLimitsForward(float J1angle, float J2angle, float J3distance, float J4angle) // has to be checked
{
  int forwardKinematicError = 1;
  // check if the solution is possible
  if ((J1LimNeg <= J1angle) && (J1angle <= J1LimPos) && (J2LimNeg <= J2angle) && (J2angle <= J2LimPos) && (J4LimNeg <= J4angle) && (J4LimPos >= J4angle) && (J3LimLower >= J3distance) && (J3distance >= J3LimUpper)) // LimLower = positive // LimUpper = negative
  {
    forwardKinematicError = 0;
  }
  else
  {
    forwardKinematicError = 0;
  }
  return forwardKinematicError;
}

// ================ FUNCTIONS ====================
void shiftbuffarray()
{
  if (cmdbuff1 == "")
  {
    // shift 2 to 1
    cmdbuff1 = cmdbuff2;
    cmdbuff2 = "";
  }
  if (cmdbuff2 == "")
  {
    // shift 3 to 2
    cmdbuff2 = cmdbuff3;
    cmdbuff3 = "";
  }
  if (cmdbuff1 == "")
  {
    // shift 2 to 1
    cmdbuff1 = cmdbuff2;
    cmdbuff2 = "";
  }
}
void processSerial()
{
  if (Serial.available() > 0 and cmdbuff3 == "")
  {
    char received = Serial.read();
    receivedData = receivedData + received;
    // Serial.println(receivedData);

    if (received == '\n')
    {
      // Serial.println("end of line detected");
      cmdbuff3 = receivedData;
      receivedData = "";
      shiftbuffarray();
    }
  }
}
void blinkLed()
{
  unsigned long currentMillis = millis();

  // led blink to show that the program is running
  if (currentMillis - previousMillis >= interval) // blink led
  {
    previousMillis = currentMillis;
    digitalWrite(13, !digitalRead(13)); // led is sign of life
  }
}
float getPointOnBZCurve(float percentComplete, Point start, Point control1, Point control2, Point end)
{
  float BP1 = percentComplete * percentComplete * percentComplete;                   // t^3
  float BP2 = 3 * percentComplete * percentComplete * (1 - percentComplete);         // 3t^2(1-t)
  float BP3 = 3 * percentComplete * (1 - percentComplete) * (1 - percentComplete);   // 3t(1-t)^2
  float BP4 = (1 - percentComplete) * (1 - percentComplete) * (1 - percentComplete); // (1-t)^3

  // float x = start.x * BP1 + control1.x * BP2 + control2.x * BP3 + end.x * BP4;

  float y = start.y * BP1 + control1.y * BP2 + control2.y * BP3 + end.y * BP4;
  return y;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ============================================== DRIVE AXES ===============================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DriveAxes(int J1stepsToDo, int J2stepsToDo, int J3stepsToDo, int J4stepsToDo, int J1dir, int J2dir, int J3dir, int J4dir, String controlType, float NORspeed, float ACCperc, float DCCperc, float totTime, int debug)
{
  //unsigned long period = 0;
  //unsigned long firstEdge = 0;
  //unsigned long secondEdge = 0;

  Curve speedProfileT;                // speed profile for controlling for time (do not change - the time would not be correct)
  speedProfileT.start = {5, 5};       // default values {5,5}
  speedProfileT.control1 = {42, 5};   //               {42,5}
  speedProfileT.control2 = {48, 100}; //               {48,100}
  speedProfileT.end = {100, 100};     //               {100,100}

  Curve speedProfileS;                // speed profile for controlling for speed (safe to change)
  speedProfileS.start = {0, 8};       // default values {0,3}
  speedProfileS.control1 = {42, 8};   //               {42,5}
  speedProfileS.control2 = {48, 100}; //               {48,100}
  speedProfileS.end = {100, 100};     //               {100,100}

  enum controlMode
  {
    SPEED,
    TIME
  } controlState;

  // general vars for entire function
  int pulseDelay = 5; // microseconds
  int curStepGap = 0;
  int axisMostStepscur = 0; // captures number of iterations of while loop below

  // general vars for speed control
  float MaxSpeed = 5000; // max speed in steps per second
  int stepsToAccelS = 200;
  int StepsToDecelS = 200;

  // general vars for time control
  float totalTime = 1;             // in seconds - default value
  float stepsNumberCorrection = 1; // correction for the number of steps to be done, necessary for time control
  float corrConst = 1;             // correction constant for time control
  int stepsToAccelT = 0;

  // find which axis has to do the most steps and set it as the axismoststeps
  int AxisMostSteps = J1stepsToDo;
  if (J2stepsToDo > AxisMostSteps)
  {
    AxisMostSteps = J2stepsToDo;
  }
  if (J3stepsToDo > AxisMostSteps)
  {
    AxisMostSteps = J3stepsToDo;
  }
  if (J4stepsToDo > AxisMostSteps)
  {
    AxisMostSteps = J4stepsToDo;
  }
 
  // reset
  int J1curSteps = 0;
  int J2curSteps = 0;
  int J3curSteps = 0;
  int J4curSteps = 0;

  // set directions

  // J1 //
  if (J1dir)
  {
    digitalWrite(J1dirPin, LOW);
  }
  else
  {
    digitalWrite(J1dirPin, HIGH);
  }
  // J2 //
  if (J2dir)
  {
    digitalWrite(J2dirPin, LOW);
  }
  else
  {
    digitalWrite(J2dirPin, HIGH);
  }
  // J3 //
  if (J3dir)
  {
    digitalWrite(J3dirPin, HIGH); // dolu
  }
  else
  {
    digitalWrite(J3dirPin, LOW); // nahoru
  }
  // J4 //
  if (J4dir)
  {
    digitalWrite(J4dirPin, LOW);
  }
  else
  {
    digitalWrite(J4dirPin, HIGH);
  }

  //
  if (controlType != "s" && controlType != "t" && controlType != "a") // t = controlling for time, s = controlling for speed a = automatic optimal speed and accel selection
  {
    Serial.println("Invalid control type");
    return;
  }
  if (controlType == "s")
  {
    // Serial.println("speed control");
    if (NORspeed > MaxSpeed)
    {
      Serial.println("MaxSpeed violation - max speed is over " + String(MaxSpeed) + " steps per second");
      return;
    }
    stepsToAccelS = int(float(AxisMostSteps) * (ACCperc / 100.0F)); // steps to accelerate to normal speed
    StepsToDecelS = int(float(AxisMostSteps) * (DCCperc / 100.0F));

    if (debug == 1 || debug == 2)
      Serial.println("steps to accel: " + String(stepsToAccelS) + "steps to decel: " + String(StepsToDecelS));

    controlState = SPEED;
  }
  if (controlType == "t")
  {
    // Serial.println("time control");
    totalTime = totTime;
    stepsToAccelT = AxisMostSteps / 2;                            // steps to accelerate to max speed
    stepsNumberCorrection = float(AxisMostSteps) / 16000.0F;      // necessary correction for time to be correct
    corrConst = (stepsNumberCorrection * float(700)) / totalTime; // final correction constant for time control
    if (debug == 1 || debug == 2)
    {
      Serial.println("axis most steps: " + String(AxisMostSteps) + "steps to accel: " + String(stepsToAccelT) + "stepsNumberCorrection: " + String(stepsNumberCorrection) + "corrConst: " + String(corrConst));
    }

    controlState = TIME;
  }
  if(controlType == "a")
  {
    // Serial.println("automatic control");
    if(AxisMostSteps <= 200){
    NORspeed = 500;
    ACCperc = 25;
    DCCperc = 25;
    }
    else if(AxisMostSteps <= 500){
    NORspeed = 750;
    ACCperc = 20;
    DCCperc = 20;
    }
    else if(AxisMostSteps <= 1000){
    NORspeed = 1000;
    ACCperc = 20;
    DCCperc = 20;
    }
    else if (AxisMostSteps <= 1500){
    NORspeed = 1250;
    ACCperc = 15;
    DCCperc = 15;
    }
    else if(AxisMostSteps <= 2000){
    NORspeed = 1500;
    ACCperc = 15;
    DCCperc = 15;
    }
    else if(AxisMostSteps <= 4000){
    NORspeed = 2500;
    ACCperc = 15;
    DCCperc = 15;
    }
    else if(AxisMostSteps <= 6000){
    NORspeed = 3000;
    ACCperc = 15;
    DCCperc = 15;
    }
    else if(AxisMostSteps <= 15000){
    NORspeed = 3000;
    ACCperc = 10;
    DCCperc = 10;
    }
    Serial.println("Automatic NORSpeed: " + String(NORspeed) + " ACCperc: " + String(ACCperc) + " DCCperc: " + String(DCCperc));
    stepsToAccelS = int(float(AxisMostSteps) * (ACCperc / 100.0F)); // steps to accelerate to normal speed
    StepsToDecelS = int(float(AxisMostSteps) * (DCCperc / 100.0F));
    controlState = SPEED;
  }

  // ----------------- calculate how often to make steps, step skips----------------- //
  // ------------------ necessary for synchronization of the axes ------------------ //
  int J1_StepOnIter = 0;
  int J2_StepOnIter = 0;
  int J3_StepOnIter = 0;
  int J4_StepOnIter = 0;

  // first layer of step skipping, undone steps are used to calculate, how often to skip
  int J1_SkipOnIter1 = 0;
  int J2_SkipOnIter1 = 0;
  int J3_SkipOnIter1 = 0;
  int J4_SkipOnIter1 = 0;

  int J1_UndoneSteps1 = 0;
  int J2_UndoneSteps1 = 0;
  int J3_UndoneSteps1 = 0;
  int J4_UndoneSteps1 = 0;

  // second layer of step skipping
  int J1_SkipOnIter2 = 0;
  int J2_SkipOnIter2 = 0;
  int J3_SkipOnIter2 = 0;
  int J4_SkipOnIter2 = 0;

  int J1_UndoneSteps2 = 0;
  int J2_UndoneSteps2 = 0;
  int J3_UndoneSteps2 = 0;
  int J4_UndoneSteps2 = 0;

  // reset
  int J1cur_StepOnIter = 0;
  int J2cur_StepOnIter = 0;
  int J3cur_StepOnIter = 0;
  int J4cur_StepOnIter = 0;

  int J1cur_SkipOnIter1 = 0;
  int J2cur_SkipOnIter1 = 0;
  int J3cur_SkipOnIter1 = 0;
  int J4cur_SkipOnIter1 = 0;

  int J1cur_SkipOnIter2 = 0;
  int J2cur_SkipOnIter2 = 0;
  int J3cur_SkipOnIter2 = 0;
  int J4cur_SkipOnIter2 = 0;

  //-------J1 - StepOnIter, SkipOnIter1, SkipOnIter2-------//
  J1_StepOnIter = (AxisMostSteps / J1stepsToDo); // integer division always rounds down but is a lot faster than float division
  J1_UndoneSteps1 = (AxisMostSteps - (J1stepsToDo * J1_StepOnIter));
  if (J1_UndoneSteps1 > 0)
  {
    J1_SkipOnIter1 = (AxisMostSteps / J1_UndoneSteps1);
  }
  else
  {
    J1_SkipOnIter1 = 0;
  }
  if (J1_SkipOnIter1 > 0)
  {
    J1_UndoneSteps2 = (AxisMostSteps - (J1stepsToDo * J1_StepOnIter) - ((J1stepsToDo * J1_StepOnIter) / J1_SkipOnIter1));
  }
  else
  {
    J1_UndoneSteps2 = 0;
  }
  if (J1_UndoneSteps2 > 0)
  {
    J1_SkipOnIter2 = (AxisMostSteps / J1_UndoneSteps2);
  }
  else
  {
    J1_SkipOnIter2 = 0;
  }

  if (debug == 1 || debug == 2)
  {
    Serial.println("vars const 1: J1_StepOnIter: " + String(J1_StepOnIter) + " J1_SkipOnIter1: " + String(J1_SkipOnIter1) + " J1_SkipOnIter2: " + String(J1_SkipOnIter2));
  }

  //-------J2 - StepOnIter, SkipOnIter1, SkipOnIter2-------//
  J2_StepOnIter = (AxisMostSteps / J2stepsToDo); // integer division always rounds down but is a lot faster than float division
  J2_UndoneSteps1 = (AxisMostSteps - (J2stepsToDo * J2_StepOnIter));
  if (J2_UndoneSteps1 > 0)
  {
    J2_SkipOnIter1 = (AxisMostSteps / J2_UndoneSteps1);
  }
  else
  {
    J2_SkipOnIter1 = 0;
  }
  if (J2_SkipOnIter1 > 0)
  {
    J2_UndoneSteps2 = (AxisMostSteps - (J2stepsToDo * J2_StepOnIter) - ((J2stepsToDo * J2_StepOnIter) / J2_SkipOnIter1));
  }
  else
  {
    J2_UndoneSteps2 = 0;
  }
  if (J2_UndoneSteps2 > 0)
  {
    J2_SkipOnIter2 = (AxisMostSteps / J2_UndoneSteps2);
  }
  else
  {
    J2_SkipOnIter2 = 0;
  }

  if (debug == 1 || debug == 2)
  {
    Serial.println("vars const 2: J2_StepOnIter: " + String(J2_StepOnIter) + " J2_SkipOnIter1: " + String(J2_SkipOnIter1) + " J2_SkipOnIter2: " + String(J2_SkipOnIter2));
  }

  //-------J3 - StepOnIter, SkipOnIter1, SkipOnIter2-------//
  J3_StepOnIter = (AxisMostSteps / J3stepsToDo); // integer division always rounds down but is a lot faster than float division
  J3_UndoneSteps1 = (AxisMostSteps - (J3stepsToDo * J3_StepOnIter));
  if (J3_UndoneSteps1 > 0)
  {
    J3_SkipOnIter1 = (AxisMostSteps / J3_UndoneSteps1);
  }
  else
  {
    J3_SkipOnIter1 = 0;
  }
  if (J3_SkipOnIter1 > 0)
  {
    J3_UndoneSteps2 = (AxisMostSteps - (J3stepsToDo * J3_StepOnIter) - ((J3stepsToDo * J3_StepOnIter) / J3_SkipOnIter1));
  }
  else
  {
    J3_UndoneSteps2 = 0;
  }
  if (J3_UndoneSteps2 > 0)
  {
    J3_SkipOnIter2 = (AxisMostSteps / J3_UndoneSteps2);
  }
  else
  {
    J3_SkipOnIter2 = 0;
  }

  if (debug == 1 || debug == 2)
  {
    Serial.println("vars const 3: J3_StepOnIter: " + String(J3_StepOnIter) + " J3_SkipOnIter1: " + String(J3_SkipOnIter1) + " J3_SkipOnIter2: " + String(J3_SkipOnIter2));
  }

  //-------J4 - StepOnIter, SkipOnIter1, SkipOnIter2-------//
  J4_StepOnIter = (AxisMostSteps / J4stepsToDo); // integer division always rounds down but is a lot faster than float division
  J4_UndoneSteps1 = (AxisMostSteps - (J4stepsToDo * J4_StepOnIter));
  if (J4_UndoneSteps1 > 0)
  {
    J4_SkipOnIter1 = (AxisMostSteps / J4_UndoneSteps1);
  }
  else
  {
    J4_SkipOnIter1 = 0;
  }
  if (J4_SkipOnIter1 > 0)
  {
    J4_UndoneSteps2 = (AxisMostSteps - (J4stepsToDo * J4_StepOnIter) - ((J4stepsToDo * J4_StepOnIter) / J4_SkipOnIter1));
  }
  else
  {
    J4_UndoneSteps2 = 0;
  }
  if (J4_UndoneSteps2 > 0)
  {
    J4_SkipOnIter2 = (AxisMostSteps / J4_UndoneSteps2);
  }
  else
  {
    J4_SkipOnIter2 = 0;
  }

  if (debug == 1 || debug == 2)
  {
    Serial.println("vars const 4: J4_StepOnIter: " + String(J4_StepOnIter) + " J4_SkipOnIter1: " + String(J4_SkipOnIter1) + " J4_SkipOnIter2: " + String(J4_SkipOnIter2));
  }

  ///////////////////////////////////////////////////////////////////////
  ///////////////////////////// DRIVE AXES //////////////////////////////
  ///////////////////////////////////////////////////////////////////////
  while (J1curSteps < J1stepsToDo || J2curSteps < J2stepsToDo || J3curSteps < J3stepsToDo || J4curSteps < J4stepsToDo)
  {
    //firstEdge = ARM_DWT_CYCCNT;
    if (controlState == TIME)
    {
      if (axisMostStepscur <= stepsToAccelT)
      { // accelerate
        // calculate the speed profile
        float percentComplete = (float)axisMostStepscur / (float)stepsToAccelT;
        float speed = getPointOnBZCurve(1.0F - percentComplete, speedProfileT.start, speedProfileT.control1, speedProfileT.control2, speedProfileT.end);
        speed = speed * corrConst;
        if (speed > MaxSpeed)
        {
          speed = MaxSpeed;
        }
        curStepGap = int(1000000.0F / speed);
        // Serial.println(curStepGap);
      }
      else if (axisMostStepscur <= AxisMostSteps) // decelerate
      {
        float percentComplete = (float)(axisMostStepscur - stepsToAccelT) / (float)(AxisMostSteps - stepsToAccelT);
        float speed = getPointOnBZCurve(percentComplete, speedProfileT.start, speedProfileT.control1, speedProfileT.control2, speedProfileT.end);
        speed = speed * corrConst;
        if (speed > MaxSpeed)
        {
          speed = MaxSpeed;
        }
        curStepGap = int(1000000.0F / speed);
        // Serial.println(curStepGap);
        // Serial.println("dcc");
      }
    }
    if (controlState == SPEED)
    {
      if (axisMostStepscur <= stepsToAccelS)
      { // accelerate
        // calculate the speed profile
        float percentComplete = (float)axisMostStepscur / (float)stepsToAccelS;
        float speed = (getPointOnBZCurve(1.0F - percentComplete, speedProfileS.start, speedProfileS.control1, speedProfileS.control2, speedProfileS.end) * NORspeed) / 100.0F;
        curStepGap = int(1000000.0F / speed);
        // Serial.println(speed);
        //  Serial.println("acc");
        //  Serial.println(curStepGap);
      }
      else if (axisMostStepscur <= (AxisMostSteps - StepsToDecelS)) // normal speed
      {
        // Serial.println(NORspeed);
        curStepGap = int(1000000.0F / NORspeed);
        // Serial.println("normal");
        // Serial.println(curStepGap);
      }
      else // decelerate
      {
        float percentComplete = (float)(axisMostStepscur - (AxisMostSteps - StepsToDecelS)) / (float)StepsToDecelS;
        float speed = (getPointOnBZCurve(percentComplete, speedProfileS.start, speedProfileS.control1, speedProfileS.control2, speedProfileS.end) * NORspeed) / 100.0F;
        // Serial.println(speed);
        curStepGap = int(1000000.0F / speed);
        // Serial.println("dcc");
        // Serial.println(curStepGap);
      }
    }

    /////////////////////// DRIVE J1 ///////////////////////
    if (J1curSteps < J1stepsToDo)
    {
      if (J1_SkipOnIter2 == 0)
      {
        J1cur_SkipOnIter2 = J1cur_SkipOnIter2+1;
      }
      if (J1cur_SkipOnIter2 != J1_SkipOnIter2)
      {
        J1cur_SkipOnIter2 = J1cur_SkipOnIter2+1;
        if (J1_SkipOnIter1 == 0)
        {
          J1cur_SkipOnIter1 = J1cur_SkipOnIter1+1;
        }
        if (J1cur_SkipOnIter1 != J1_SkipOnIter1)
        {
          J1cur_SkipOnIter1 = J1cur_SkipOnIter1+1;
          J1cur_StepOnIter = J1cur_StepOnIter+1;
          //if (debug == 1)
          //{
          //  Serial.println("vars cur1i: J1cur_StepOnIter: " + String(J1cur_StepOnIter) + " J1cur_SkipOnIter1: " + String(J1cur_SkipOnIter1) + " J1cur_SkipOnIter2: " + String(J1cur_SkipOnIter2) + " J1curSteps: " + String(J1curSteps));
          //}
          if (J1cur_StepOnIter == J1_StepOnIter)
          {
            J1curSteps = J1curSteps+1;
            J1cur_StepOnIter = 0;

            digitalWrite(J1stepPin, LOW);

            if (J1dir == 0)
            {
              --J1stepsMaster;
            }
            if (J1dir == 1)
            {
              ++J1stepsMaster;
            }
          }
        }
        else
        {
          J1cur_SkipOnIter1 = 0;
        }
      }
      else
      {
        J1cur_SkipOnIter2 = 0;
      }
      //if (debug == 1)
      //{
      //  Serial.println("vars cur 1: J1cur_StepOnIter: " + String(J1cur_StepOnIter) + " J1cur_SkipOnIter1: " + String(J1cur_SkipOnIter1) + " J1cur_SkipOnIter2: " + String(J1cur_SkipOnIter2) + " J1curSteps: " + String(J1curSteps));
      //  Serial.println("");
      //}
    }
    /////////////////////// DRIVE J2 ///////////////////////
    if (J2curSteps < J2stepsToDo)
    {
      if (J2_SkipOnIter2 == 0)
      {
        J2cur_SkipOnIter2 = J2cur_SkipOnIter2+1;
      }
      if (J2cur_SkipOnIter2 != J2_SkipOnIter2)
      {
        J2cur_SkipOnIter2 = J2cur_SkipOnIter2+1;
        if (J2_SkipOnIter1 == 0)
        {
          J2cur_SkipOnIter1 = J2cur_SkipOnIter1+1;
        }
        if (J2cur_SkipOnIter1 != J2_SkipOnIter1)
        {
          J2cur_SkipOnIter1 = J2cur_SkipOnIter1+1;
          J2cur_StepOnIter = J2cur_StepOnIter+1;
          //if (debug == 1)
          //{
          //  Serial.println("vars cur2i: J2cur_StepOnIter: " + String(J2cur_StepOnIter) + " J2cur_SkipOnIter1: " + String(J2cur_SkipOnIter1) + " J2cur_SkipOnIter2: " + String(J2cur_SkipOnIter2) + " J2curSteps: " + String(J2curSteps));
          //}
          if (J2cur_StepOnIter == J2_StepOnIter)
          {
            J2curSteps = J2curSteps+1;
            J2cur_StepOnIter = 0;

            digitalWrite(J2stepPin, LOW);

            if (J2dir == 0)
            {
              --J2stepsMaster;
            }
            if (J2dir == 1)
            {
              ++J2stepsMaster;
            }
          }
        }
        else
        {
          J2cur_SkipOnIter1 = 0;
        }
      }
      else
      {
        J2cur_SkipOnIter2 = 0;
      }
      //if (debug == 1)
      //{
      //  Serial.println("vars cur 2: J2cur_StepOnIter: " + String(J2cur_StepOnIter) + " J2cur_SkipOnIter1: " + String(J2cur_SkipOnIter1) + " J2cur_SkipOnIter2: " + String(J2cur_SkipOnIter2) + " J2curSteps: " + String(J2_SkipOnIter1));
      //  Serial.println("");
      //}
    }
    /////////////////////// DRIVE J3 ///////////////////////
    if (J3curSteps < J3stepsToDo)
    {
      if (J3_SkipOnIter2 == 0)
      {
        J3cur_SkipOnIter2 = J3cur_SkipOnIter2+1;
      }
      if (J3cur_SkipOnIter2 != J3_SkipOnIter2)
      {
        J3cur_SkipOnIter2 = J3cur_SkipOnIter2+1;
        if (J3_SkipOnIter1 == 0)
        {
          J3cur_SkipOnIter1 = J3cur_SkipOnIter1+1;
        }
        if (J3cur_SkipOnIter1 != J3_SkipOnIter1)
        {
          J3cur_SkipOnIter1 = J3cur_SkipOnIter1+1;
          J3cur_StepOnIter = J3cur_StepOnIter+1;
          //if (debug == 1)
          //{
          //  Serial.println("vars cur3i: J3cur_StepOnIter: " + String(J3cur_StepOnIter) + " J3cur_SkipOnIter1: " + String(J3cur_SkipOnIter1) + " J3cur_SkipOnIter2: " + String(J3cur_SkipOnIter2) + " J3curSteps: " + String(J3curSteps));
          //}
          if (J3cur_StepOnIter == J3_StepOnIter)
          {
            J3curSteps = J3curSteps+1;
            J3cur_StepOnIter = 0;

            digitalWrite(J3stepPin, LOW);
            delayMicroseconds(pulseDelay);
            digitalWrite(J3stepPin, HIGH);
            delayMicroseconds(pulseDelay);
            digitalWrite(J3stepPin, LOW);

            if (J3dir == 0)
            {
              --J3stepsMaster;
            }
            if (J3dir == 1)
            {
              ++J3stepsMaster;
            }
          }
        }
        else
        {
          J3cur_SkipOnIter1 = 0;
        }
      }
      else
      {
        J3cur_SkipOnIter2 = 0;
      }
      //if (debug == 1)
      //{
      //  Serial.println("vars cur 3: J3cur_StepOnIter: " + String(J3cur_StepOnIter) + " J3cur_SkipOnIter1: " + String(J3cur_SkipOnIter1) + " J3cur_SkipOnIter2: " + String(J3cur_SkipOnIter2) + " J3curSteps: " + String(J3curSteps));
      //  Serial.println("");
      //}
    }
    /////////////////////// DRIVE J4 ///////////////////////
    if (J4curSteps < J4stepsToDo)
    {
      if (J4_SkipOnIter2 == 0)
      {
        J4cur_SkipOnIter2 = J4cur_SkipOnIter2+1; 
      }
      if (J4cur_SkipOnIter2 != J4_SkipOnIter2)
      {
        J4cur_SkipOnIter2 = J4cur_SkipOnIter2+1;
        if (J4_SkipOnIter1 == 0)
        {
          J4cur_SkipOnIter1 = J4cur_SkipOnIter1+1;
        }
        if (J4cur_SkipOnIter1 != J4_SkipOnIter1)
        {
          J4cur_SkipOnIter1 = J4cur_SkipOnIter1+1;
          J4cur_StepOnIter  = J4cur_StepOnIter+1;
          //if (debug == 1)
          //{
          //  Serial.println("vars cur4i: J4cur_StepOnIter: " + String(J4cur_StepOnIter) + " J4cur_SkipOnIter1: " + String(J4cur_SkipOnIter1) + " J4cur_SkipOnIter2: " + String(J4cur_SkipOnIter2) + " J4curSteps: " + String(J4curSteps));
          //}
          if (J4cur_StepOnIter == J4_StepOnIter)
          {
            J4curSteps = J4curSteps+1;
            J4cur_StepOnIter = 0;

            digitalWrite(J4stepPin, LOW);

            if (J4dir == 0)
            {
              --J4stepsMaster;
            }
            if (J4dir == 1)
            {
              ++J4stepsMaster;
            }
          }
        }
        else
        {
          J4cur_SkipOnIter1 = 0;
        }
      }
      else
      {
        J4cur_SkipOnIter2 = 0;
      }

      //if (debug == 1)
      //{
      //  Serial.println("vars cur 4: J4cur_StepOnIter: " + String(J4cur_StepOnIter) + " J4cur_SkipOnIter1: " + String(J4cur_SkipOnIter1) + " J4cur_SkipOnIter2: " + String(J4cur_SkipOnIter2) + " J4curSteps: " + String(J4curSteps));
      //  Serial.println("");
      //}
    }

    // end pulse for all axes
    delayMicroseconds(pulseDelay);
    digitalWrite(J1stepPin, HIGH);
    digitalWrite(J2stepPin, HIGH);
    digitalWrite(J3stepPin, HIGH);
    digitalWrite(J4stepPin, HIGH);
    ++axisMostStepscur;
    if(J1stepsMaster > J1MaxStepsPos || J1stepsMaster < J1StepsDef|| J2stepsMaster > J2MaxStepsPos || J2stepsMaster < J2StepsDef || J3stepsMaster > J3MaxStepsPos || J3stepsMaster < J3StepsDef || J4stepsMaster > J4MaxStepsPos || J4stepsMaster < J4StepsDef){
      Serial.println("Fatal error - robot tried to move over axis limits");
      break;
    }
    //secondEdge = ARM_DWT_CYCCNT;
    //period = secondEdge - firstEdge;
    //Serial.println(period);
    delayMicroseconds(curStepGap - pulseDelay -5);
  }

  Serial.println("ACKRP A" + String(J1stepsMaster) + " B" + String(J2stepsMaster) + " C" + String(J3stepsMaster) + " D" + String(J4stepsMaster));
  Serial.println("DriveAxes END");
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ============================================ DRIVE MOTORS J END =============================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ================ ----- ====================
// ================ SETUP ====================
// ================ ----- ====================

void setup()
{
  Serial.begin(9600);
  delay(200);
  Serial.println("Starting up");
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(J1stepPin, OUTPUT);
  pinMode(J1dirPin, OUTPUT);
  pinMode(J2stepPin, OUTPUT);
  pinMode(J2dirPin, OUTPUT);
  pinMode(J3stepPin, OUTPUT);
  pinMode(J3dirPin, OUTPUT);
  pinMode(J4stepPin, OUTPUT);
  pinMode(J4dirPin, OUTPUT);

  pinMode(J1enablePin, OUTPUT);
  pinMode(J2enablePin, OUTPUT);
  pinMode(J3enablePin, OUTPUT);
  pinMode(J4enablePin, OUTPUT);

  pinMode(J1calPin, INPUT);
  pinMode(J2calPin, INPUT);
  pinMode(J3calPin, INPUT);
  pinMode(J4calPin, INPUT_PULLDOWN);

  TeensyState = IDLE;

  cmdbuff3 = "";
  cmdbuff2 = "";
  cmdbuff1 = "";

  digitalWrite(LED_BUILTIN, HIGH);

  delay(1000);
  Serial.println("Initialization complete");
}

void loop()
{
  blinkLed();
  processSerial();

  switch (TeensyState)
  {
  // ========================== IDLE ==========================
  case IDLE:
  {
    if (cmdbuff1 != "")
    {
      TeensyState = PROCESSING_INSTRUCTION;
      break;
    }
  }
  break;
  // ================== PROCESSING INSTRUCTION ==================
  case PROCESSING_INSTRUCTION:
  {
    processedData = cmdbuff1;                    // copy the string from the buffer1 to currently processed string
    processedData.trim();                        // remove whitespaces from the string
    instruction = processedData.substring(0, 2); // get the first 2 characters of the string (the instruction)
    processedData = processedData.substring(3);

    if (instruction == "DL")
    {
      if (debug == 1 || debug == 2)
        Serial.println("delay instruction detected");
      TeensyState = DL;
      break;
    }
    if (instruction == "HS")
    {
      if (debug == 1 || debug == 2)
        Serial.println("home to limit switches instruction detected");
      TeensyState = HS;
      break;
    }
    if (instruction == "EN")
    {
      if (debug == 1 || debug == 2)
        Serial.println("enable motors instruction detected");
      TeensyState = EN;
      break;
    }
    if (instruction == "KF")
    {
      if (debug == 1 || debug == 2)
        Serial.println("forward kinematics instruction detected");
      TeensyState = KF;
      break;
    }
    if (instruction == "JJ")
    {
      if (debug == 1 || debug == 2)
        Serial.println("jog joints instruction detected");
      TeensyState = JJ;
      break;
    }
    if (instruction == "MJ")
    {
      if (debug == 1 || debug == 2)
        Serial.println("MOVEJ instruction detected");
      TeensyState = MJ;
      break;
    }
    if (instruction == "TL")
    {
      if (debug == 1 || debug == 2)
        Serial.println("test limit switches instruction detected");
      TeensyState = TL;
      break;
    }
    if (instruction == "TD")
    {
      if (debug == 1 || debug == 2)
        Serial.println("test drive function instruction detected");
      TeensyState = TD;
      break;
    }
    if (instruction == "RP")
    {
      if (debug == 1 || debug == 2)
        Serial.println("send robot position instruction detected");
      TeensyState = RP;
      break;
    }
    else
    {
      Serial.println("no valid instruction detected, going idle");
      TeensyState = CLEARING_BUFFER1;
      break;
    }
  }
  break;

  // ================== DELAY FOR TIME ==================
  case DL:
  {
    int TimeStartIndex = processedData.indexOf("T");
    float waitingTime = processedData.substring(TimeStartIndex + 1).toFloat();   // get the value after the T character and convert it to float
    waitingTime = waitingTime * 1000;                                            // convert the value to milliseconds
    delay(waitingTime);                                                          // delay for that amount of time
    Serial.println("ACKDL delay for " + String(waitingTime / 1000) + " seconds done"); // print the delay time
    TeensyState = CLEARING_BUFFER1;
  }
  break;

  // ================== TEST LIMIT SWITCHES ==================
  case TL:
  {
    for (int i = 0; i < 5000; i++)
    {

      if (digitalRead(J1calPin) == HIGH)
      {
        Serial.println(F("J1 limit switch triggered"));
      }
      if (digitalRead(J2calPin) == HIGH)
      {
        Serial.println(F("J2 limit switch triggered"));
      }
      if (digitalRead(J3calPin) == HIGH)
      {
        Serial.println(F("J3 limit switch triggered"));
      }
      if (digitalRead(J4calPin) == HIGH)
      {
        Serial.println(F("J4 limit switch triggered"));
      }
      delay(5);
    }
    TeensyState = CLEARING_BUFFER1;
  }
  break;

  // ================== ENABLE MOTORS ==================
  case EN:
  {
    Serial.println(processedData);
    int indexJ1 = processedData.indexOf('A');
    int indexJ2 = processedData.indexOf('B');
    int indexJ3 = processedData.indexOf('C');
    int indexJ4 = processedData.indexOf('D');

    int J1enable = processedData.substring(indexJ1 + 1, indexJ1 + 2).toInt();
    int J2enable = processedData.substring(indexJ2 + 1, indexJ2 + 2).toInt();
    int J3enable = processedData.substring(indexJ3 + 1, indexJ3 + 2).toInt();
    int J4enable = processedData.substring(indexJ4 + 1, indexJ4 + 2).toInt();
    String sendBack = "ACKEN A" + String(J1enable) + "B" + String(J2enable) + "C" + String(J3enable) + "D" + String(J4enable);
    Serial.println(sendBack);
    if (J1enable == 1)
    {
      digitalWrite(J1enablePin, HIGH);
    }
    else
    {
      digitalWrite(J1enablePin, LOW);
    }
    if (J2enable == 1)
    {
      digitalWrite(J2enablePin, HIGH);
    }
    else
    {
      digitalWrite(J2enablePin, LOW);
    }
    if (J3enable == 1)
    {
      digitalWrite(J3enablePin, HIGH);
    }
    else
    {
      digitalWrite(J3enablePin, LOW);
    }
    if (J4enable == 1)
    {
      digitalWrite(J4enablePin, HIGH);
    }
    else
    {
      digitalWrite(J4enablePin, LOW);
    }
    TeensyState = CLEARING_BUFFER1;
  }
  break;

  // ================== HOME TO LIMIT SWITCHES INIT==================
  case HS:
  {
    Serial.println(processedData);

    int indexJ1 = processedData.indexOf('A');
    int indexJ2 = processedData.indexOf('B');
    int indexJ3 = processedData.indexOf('C');
    int indexJ4 = processedData.indexOf('D');
    int SPstart = processedData.indexOf('S');

    int J1home = processedData.substring(indexJ1 + 1, indexJ1 + 2).toInt();
    int J2home = processedData.substring(indexJ2 + 1, indexJ2 + 2).toInt();
    int J3home = processedData.substring(indexJ3 + 1, indexJ3 + 2).toInt();
    int J4home = processedData.substring(indexJ4 + 1, indexJ4 + 2).toInt();
    int SpeedIn = processedData.substring(SPstart + 1).toInt();

    //  RESET COUNTERS
    int J1stepsDone = 0;
    int J2stepsDone = 0;
    int J3stepsDone = 0;
    int J4stepsDone = 0;

    int J1pass = 0;
    int J2pass = 0;
    int J3pass = 0;
    int J4pass = 0;
    int J3homed = 0;
    int iter1 = 0;
    int speed = 0;

    if (J1home == 0 && J2home == 0 && J3home == 0 && J4home == 0)
    {
      Serial.println(F("No axis selected for homing, going idle"));
      TeensyState = CLEARING_BUFFER1;
      break;
    }
    if (SpeedIn == 0)
    {
      Serial.println(F("no speed set, going idle"));
      TeensyState = CLEARING_BUFFER1;
      break;
    }
    if (SpeedIn > 500)
    {
      Serial.println(F("speed too high (>500), going idle"));
      TeensyState = CLEARING_BUFFER1;
      break;
    }

    if (J2home || J1home || J4home)
    {
      J3home = 1; // if J1 J2 or J4 is selected for homing, J3 will be homed as well (to avoid collision with the connector panel)
    }
    int stepTimeCalc = (1000000 / SpeedIn);
    if (debug == 1 || debug == 2)
      Serial.println(stepTimeCalc);

    // SET DIRECTIONS
    //  J1 //
    if (J1caldir == 1)
    {
      digitalWrite(J1dirPin, LOW);
    }
    else if (J1caldir == 0)
    {
      digitalWrite(J1dirPin, HIGH);
    }

    // J2 //
    if (J2caldir == 1)
    {
      digitalWrite(J2dirPin, LOW);
    }
    else if (J2caldir == 0)
    {
      digitalWrite(J2dirPin, HIGH);
    }
    // J3 //
    if (J3caldir == 1)
    {
      digitalWrite(J3dirPin, LOW);
    }
    else if (J3caldir == 0)
    {
      digitalWrite(J3dirPin, HIGH);
    }

    // J4 //
    if (J4caldir == 1)
    {
      digitalWrite(J4dirPin, LOW);
    }
    else if (J4caldir == 0)
    {
      digitalWrite(J4dirPin, HIGH);
    }

    delayMicroseconds(50);

    while (!((J1home + J2home + J3home + J4home) * 4 == J1pass + J2pass + J3pass + J4pass))
    {
      // drive the motors until limit switches are triggered at least 4 times in a row (to avoid false triggering)
      // J1 //
      if (J1pass < 4 && J1home && J3homed)
      {
        if (digitalRead(J1calPin) == LOW)
        {
          digitalWrite(J1stepPin, LOW);
          delayMicroseconds(2);
          digitalWrite(J1stepPin, HIGH);
          J1pass = 0;
          J1stepsDone++;
        }
        J1pass++;
      }
      // J2 //
      if (J2pass < 4 && J2home && J3homed)
      {
        if (digitalRead(J2calPin) == LOW)
        {
          digitalWrite(J2stepPin, LOW);
          delayMicroseconds(2);
          digitalWrite(J2stepPin, HIGH);
          J2pass = 0;
          J2stepsDone++;
        }
        J2pass++;
      }
      // J3 //
      if (J3pass < 4 && J3home)
      {
        if (digitalRead(J3calPin) == LOW)
        {
          digitalWrite(J3stepPin, LOW);
          delayMicroseconds(6);
          digitalWrite(J3stepPin, HIGH);
          delayMicroseconds(6);
          digitalWrite(J3stepPin, LOW);
          delayMicroseconds(6);
          digitalWrite(J3stepPin, HIGH);
          delayMicroseconds(6);
          J3pass = 0;
          J3stepsDone++;
        }
        J3pass++;
      }
      // J4 //
      if (J4pass < 4 && J4home && J3homed)
      {
        if (digitalRead(J4calPin) == LOW)
        {
          digitalWrite(J4stepPin, LOW);
          delayMicroseconds(2);
          digitalWrite(J4stepPin, HIGH);
          J4pass = 0;
          J4stepsDone++;
        }
        J4pass++;
      }

      // speed settings//
      speed = 15000 - iter1 * 35; // iter1 constant is rate of acceleration

      if (J3pass == 4 && J3homed == 0)
      {
        iter1 = 0;
        J3homed = 1;
        delay(1000);
      }
      if (J3homed == 0)
      {
        speed = (stepTimeCalc / 2);
      }
      if (J3homed == 1 && speed >= stepTimeCalc)
      {
        iter1++;
      }
      delayMicroseconds(speed); // this delay determines the speed of the motors - the lower the number the faster the motors will run
    }
    if (debug == 1 || debug == 2)
    {
      Serial.println(iter1);
      Serial.println(F(" J1pass:") + String(J1pass) + F(" J2pass:") + String(J2pass) + F(" J3pass:") + String(J3pass) + F(" J4pass:") + String(J4pass));
    }
    // change the direction of the motors
    if (J1caldir == 1)
    {
      digitalWrite(J1dirPin, HIGH);
    }
    else if (J1caldir == 0)
    {
      digitalWrite(J1dirPin, LOW);
    }

    // J2 //
    if (J2caldir == 1)
    {
      digitalWrite(J2dirPin, HIGH);
    }
    else if (J2caldir == 0)
    {
      digitalWrite(J2dirPin, LOW);
    }

    // J3 //
    if (J3caldir == 1)
    {
      digitalWrite(J3dirPin, HIGH);
    }
    else if (J3caldir == 0)
    {
      digitalWrite(J3dirPin, LOW);
    }

    // J4 //
    if (J4caldir == 1)
    {
      digitalWrite(J4dirPin, HIGH);
    }
    else if (J4caldir == 0)
    {
      digitalWrite(J4dirPin, LOW);
    }

    J1pass = 0;
    J2pass = 0;
    J3pass = 0;
    J4pass = 0;

    delay(500);
    // step back from the limit switches
    if (debug == 1 || debug == 2)
    {
      Serial.println(J1home + J2home + J3home + J4home);
      Serial.println(J1pass + J2pass + J3pass + J4pass);
      Serial.println("J1 steps done: " + String(J1stepsDone) + " J2 steps done: " + String(J2stepsDone) + " J3 steps done: " + String(J3stepsDone) + " J4 steps done: " + String(J4stepsDone));
    }
    while (!((J1home + J2home + J3home + J4home) == (J1pass + J2pass + J3pass + J4pass))) //(J1pass == 0 || J2pass == 0 || J3pass == 0 || J4pass == 0)
    {
      // J1
      if (J1pass == 0 && J1home)
      {
        if (digitalRead(J1calPin) == HIGH)
        {
          digitalWrite(J1stepPin, LOW);
          delayMicroseconds(5);
          digitalWrite(J1stepPin, HIGH);
          J1stepsDone--;
        }
        if (digitalRead(J1calPin) == LOW)
        {
          J1pass = 1;
          Serial.println("J1pass ");
          J1stepsMaster = J1StepsDef;
        }
      }
      // J2
      if (J2pass == 0 && J2home)
      {
        if (digitalRead(J2calPin) == HIGH)
        {
          digitalWrite(J2stepPin, LOW);
          delayMicroseconds(5);
          digitalWrite(J2stepPin, HIGH);
          J2stepsDone--;
        }
        if (digitalRead(J2calPin) == LOW)
        {
          J2pass = 1;
          Serial.println("J2pass ");
          J2stepsMaster = J2StepsDef;
        }
      }

      // J3
      if (J3pass == 0 && J3home)
      {
        if (digitalRead(J3calPin) == HIGH)
        {
          digitalWrite(J3stepPin, LOW);
          delayMicroseconds(7);
          digitalWrite(J3stepPin, HIGH);
          J3stepsDone--;
        }
        if (digitalRead(J3calPin) == LOW)
        {
          J3pass = 1;
          Serial.println("J3pass ");
          J3stepsMaster = J3StepsDef;
        }
      }

      // J4
      if (J4pass == 0 && J4home)
      {
        if (digitalRead(J4calPin) == HIGH)
        {
          digitalWrite(J4stepPin, LOW);
          delayMicroseconds(5);
          digitalWrite(J4stepPin, HIGH);
          J4stepsDone--;
        }
        if (digitalRead(J4calPin) == LOW)
        {
          J4pass = 1;
          Serial.println("J4pass ");
          J4stepsMaster = J4StepsDef;
        }
      }
      delayMicroseconds(stepTimeCalc * 8); // this delay determines the speed of the motors - the lower the number the faster the motors will run
    }

    Serial.println("Robot homed to selected limit switches");
    Serial.println("ACKHS A " + String(J1stepsDone) + " B " + String(J2stepsDone) + " C " + String(J3stepsDone) + " D " + String(J4stepsDone));
    TeensyState = CLEARING_BUFFER1;
  }
  break;

  // ================== FORWARD KINEMATICS TEST ==================
  case KF:
  {
    Serial.println(processedData);
    int indexJ1 = processedData.indexOf('A');
    int indexJ2 = processedData.indexOf('B');
    int indexJ3 = processedData.indexOf('C');
    int indexJ4 = processedData.indexOf('D');

    float J1angle = processedData.substring(indexJ1 + 1, indexJ2).toFloat();
    float J2angle = processedData.substring(indexJ2 + 1, indexJ3).toFloat();
    float J3distance = processedData.substring(indexJ3 + 1, indexJ4).toFloat();
    float J4angle = processedData.substring(indexJ4 + 1).toFloat();
    Serial.println("J1: " + String(J1angle, 4) + " J2: " + String(J2angle) + " J3: " + String(J3distance) + " J4: " + String(J4angle));
    ForwardKinematicsSolver(J1angle, J2angle, J3distance, J4angle);
    Serial.println("x: " + String(fw_x, 4) + " y: " + String(fw_y, 4) + " z: " + String(fw_z, 4) + " fi: " + String(fw_fi, 4));
    Serial.println(checkLimitsForward(J1angle, J2angle, J3distance, J4angle));
    TeensyState = CLEARING_BUFFER1;
  }
  break;

  // test drive function
  case TD:
  {
    int indexJ1 = processedData.indexOf('A');
    int indexJ2 = processedData.indexOf('B');
    int indexJ3 = processedData.indexOf('C');
    int indexJ4 = processedData.indexOf('D');

    int J1steps1 = processedData.substring(indexJ1 + 1, indexJ2).toInt();
    int J1dir1 = processedData.substring(indexJ2 + 1, indexJ3).toInt();
    float J1speed = processedData.substring(indexJ3 + 1, indexJ4).toFloat();
    int J1acc1 = processedData.substring(indexJ4 + 1).toFloat();
    Serial.println("J1steps1: " + String(J1steps1) + " J1dir1: " + String(J1dir1) + " J1speed: " + String(J1speed) + " J1acc1: " + String(J1acc1));
    DriveAxes(J1steps1, 0, 0, 0, J1dir1, 0, 0, 0, "s", J1speed, J1acc1, J1acc1, 0, 0);
    TeensyState = CLEARING_BUFFER1;
  }
  break;

  // ================== JOG JOINTS ==================
  case JJ:
  {
    // Serial.println(processedData);
    int indexJ1 = processedData.indexOf('A');
    int indexJ2 = processedData.indexOf('B');
    int indexJ3 = processedData.indexOf('C');
    int indexJ4 = processedData.indexOf('D');
    int indexControlType = processedData.indexOf('E');
    int indexNORSpeed = processedData.indexOf('F');
    int indexACCperc = processedData.indexOf('G');
    int indexDCCperc = processedData.indexOf('H');
    int indexTotTime = processedData.indexOf('I');
    int indexDebug = processedData.indexOf('J');

    float J1angle = processedData.substring(indexJ1 + 1, indexJ2).toFloat();
    float J2angle = processedData.substring(indexJ2 + 1, indexJ3).toFloat();
    float J3distance = processedData.substring(indexJ3 + 1, indexJ4).toFloat();
    float J4angle = processedData.substring(indexJ4 + 1).toFloat();
    String controlType = processedData.substring(indexControlType + 1, indexControlType + 2);
    float NORSpeed = processedData.substring(indexNORSpeed + 1, indexACCperc).toFloat();
    float ACCperc = processedData.substring(indexACCperc + 1, indexDCCperc).toFloat();
    float DCCperc = processedData.substring(indexDCCperc + 1, indexTotTime).toFloat();
    float TotTime = processedData.substring(indexTotTime + 1, indexDebug).toFloat();
    int debug = processedData.substring(indexDebug + 1, indexDebug + 2).toInt();

    Serial.println("J1: " + String(J1angle, 4) + " J2: " + String(J2angle) + " J3: " + String(J3distance) + " J4: " + String(J4angle));
    ForwardKinematicsSolver(J1angle, J2angle, J3distance, J4angle);
    Serial.println("x: " + String(fw_x, 4) + " y: " + String(fw_y, 4) + " z: " + String(fw_z, 4) + " fi: " + String(fw_fi, 4));
    if (checkLimitsForward(J1angle, J2angle, J3distance, J4angle) == 1)
    {
      Serial.println("Jog is out of robot axis limits");
      TeensyState = CLEARING_BUFFER1;
      break;
    }
    int J1actualPosSteps = J1stepsMaster;
    int J2actualPosSteps = J2stepsMaster;
    int J3actualPosSteps = J3stepsMaster;
    int J4actualPosSteps = J4stepsMaster;

    int J1targetPosSteps = int(J1angle * J1stepsPerDeg);
    int J2targetPosSteps = int(J2angle * J2stepsPerDeg);
    int J3targetPosSteps = int(J3distance * J3stepsPerMM);
    int J4targetPosSteps = int(J4angle * J4stepsPerDeg);

    int J1stepsToDo = J1targetPosSteps - J1actualPosSteps;
    int J2stepsToDo = J2targetPosSteps - J2actualPosSteps;
    int J3stepsToDo = J3targetPosSteps - J3actualPosSteps;
    int J4stepsToDo = J4targetPosSteps - J4actualPosSteps;

    int J1dir = 0; // 0 = negative, 1 = positive
    int J2dir = 0; // 0 = negative, 1 = positive
    int J3dir = 0; // 1 = negative, 0 = positive
    int J4dir = 0; // 0 = negative, 1 = positive

    if (J1stepsToDo == 0 && J2stepsToDo == 0 && J3stepsToDo == 0 && J4stepsToDo == 0)
    {
      Serial.println("Joints are already in the desired position");
      TeensyState = CLEARING_BUFFER1;
      break;
    }
    Serial.println("J1stepsToDo: " + String(J1stepsToDo) + " J2stepsToDo: " + String(J2stepsToDo) + " J3stepsToDo: " + String(J3stepsToDo) + " J4stepsToDo: " + String(J4stepsToDo));
    if (J1stepsToDo > 0)
    {
      J1dir = 1;
    }
    if (J2stepsToDo > 0)
    {
      J2dir = 1;
    }
    if (J3stepsToDo > 0)
    {
      J3dir = 1;
    }
    if (J4stepsToDo > 0)
    {
      J4dir = 1;
    }
    J1stepsToDo = abs(J1stepsToDo);
    J2stepsToDo = abs(J2stepsToDo);
    J3stepsToDo = abs(J3stepsToDo);
    J4stepsToDo = abs(J4stepsToDo);

    DriveAxes(J1stepsToDo, J2stepsToDo, J3stepsToDo, J4stepsToDo, J1dir, J2dir, J3dir, J4dir, controlType, NORSpeed, ACCperc, DCCperc, TotTime, debug);

    TeensyState = CLEARING_BUFFER1;
  }
  break;

    // ======================== MOVE J ========================
  case MJ:
  {
    // Serial.println(processedData);
    int indexX = processedData.indexOf('X');
    int indexY = processedData.indexOf('Y');
    int indexZ = processedData.indexOf('Z');
    int indexFi = processedData.indexOf('F');
    int indexSolution = processedData.indexOf('H');
    int indexControlType = processedData.indexOf('C');
    int indexNORSpeed = processedData.indexOf('S');
    int indexACCperc = processedData.indexOf('A');
    int indexDCCperc = processedData.indexOf('D');
    int indexTotTime = processedData.indexOf('T');
    int indexDebug = processedData.indexOf('B');

    float x = processedData.substring(indexX + 1, indexY).toFloat();
    float y = processedData.substring(indexY + 1, indexZ).toFloat();
    float z = processedData.substring(indexZ + 1, indexFi).toFloat();
    float fi = processedData.substring(indexFi + 1, indexSolution).toFloat();
    String solutionType = processedData.substring(indexSolution + 1, indexSolution + 2);
    String controlType = processedData.substring(indexControlType + 1, indexControlType + 2);
    float NORSpeed = processedData.substring(indexNORSpeed + 1, indexACCperc).toFloat();
    float ACCperc = processedData.substring(indexACCperc + 1, indexDCCperc).toFloat();
    float DCCperc = processedData.substring(indexDCCperc + 1, indexTotTime).toFloat();
    float TotTime = processedData.substring(indexTotTime + 1, indexDebug).toFloat();
    int debug = processedData.substring(indexDebug + 1, indexDebug + 2).toInt();
    Serial.println("______________________");
    Serial.println("X: " + String(x, 2) + " Y: " + String(y, 2) + " Z: " + String(z, 2) + " Fi: " + String(fi, 2) + " Solution: " + solutionType + " Control: " + controlType + " NORSpeed: " + String(NORSpeed, 2) + " ACCperc: " + String(ACCperc, 2) + " DCCperc: " + String(DCCperc, 2) + " TotTime: " + String(TotTime, 2) + " Debug: " + String(debug, 2)); 
    InverseKinematicsSolver(x, y, z, fi);
    if (kinError)
    {
      Serial.println("Inverse kinematics error - point is out of robot workspace");
      TeensyState = CLEARING_BUFFER1;
      break;
    }
    Serial.println("J1left: " + String(inv_J1deg[0], 4) + " J2left: " + String(inv_J2deg[0], 4) + " J3distance: " + String(inv_J3distance, 4) + " J4left: " + String(inv_J4deg[0], 4));
    Serial.println("J1right: " + String(inv_J1deg[1], 4) + " J2right: " + String(inv_J2deg[1], 4) + " J3distance: " + String(inv_J3distance, 4) + " J4right: " + String(inv_J4deg[1], 4));
    checkLimitsInverse();

    int J1stepsToDo[2] = {int(inv_J1deg[0] * J1stepsPerDeg) - J1stepsMaster, int(inv_J1deg[1] * J1stepsPerDeg) - J1stepsMaster};
    int J2stepsToDo[2] = {int(inv_J2deg[0] * J2stepsPerDeg) - J2stepsMaster, int(inv_J2deg[1] * J2stepsPerDeg) - J2stepsMaster};
    int J3stepsToDo[2] = {int(inv_J3distance * J3stepsPerMM) - J3stepsMaster, int(inv_J3distance * J3stepsPerMM) - J3stepsMaster};
    int J4stepsToDo[2] = {int(inv_J4deg[0] * J4stepsPerDeg) - J4stepsMaster, int(inv_J4deg[1] * J4stepsPerDeg) - J4stepsMaster};

    int J1dir = 0; // 0 = negative, 1 = positive
    int J2dir = 0; // 0 = negative, 1 = positive
    int J3dir = 0; // 0 = negative, 1 = positive // 1 is not for J3 negative because the direction is inverted in DriveAxes function
    int J4dir = 0; // 0 = negative, 1 = positive

    int sol = 10; // 0 = left, 1 = right , 10 = no solution selected yet

    if (solutionType == "l")
    {
      if (leftHandedPossible)
      {
        sol = 0;
      }
      else
      {
        Serial.println("Left handed solution not possible, going idle");
        TeensyState = CLEARING_BUFFER1;
        break;
      }
    }

    if (solutionType == "r")
    {
      if (rightHandedPossible)
      {
        sol = 1;
      }
      else
      {
        Serial.println("Right handed solution not possible, going idle");
        TeensyState = CLEARING_BUFFER1;
        break;
      }
    }

    if (solutionType == "b")
    {
      if (leftHandedPossible && rightHandedPossible)
      {
        if ((abs(J1stepsToDo[0]) + abs(J2stepsToDo[0]) + abs(J3stepsToDo[0]) + abs(J4stepsToDo[0])) < (abs(J1stepsToDo[1]) + abs(J2stepsToDo[1]) + abs(J3stepsToDo[1]) + abs(J4stepsToDo[1])))
        {
          sol = 0;
        }
        else
        {
          sol = 1;
        }
      }
      else if (leftHandedPossible)
      {
        sol = 0;
      }
      else if (rightHandedPossible)
      {
        sol = 1;
      }
      else
      {
        Serial.println("No solution possible, going idle");
        TeensyState = CLEARING_BUFFER1;
        break;
      }
    }

    if (sol == 10)
    {
      Serial.println("Error: no solution selected, going idle");
      TeensyState = CLEARING_BUFFER1;
      break;
    }

    if (J1stepsToDo[sol] == 0 && J2stepsToDo[sol] == 0 && J3stepsToDo[sol] == 0 && J4stepsToDo[sol] == 0)
    {
      Serial.println("Joints are already in the desired position");
      TeensyState = CLEARING_BUFFER1;
      break;
    }
    Serial.println("J1stepsToDo: " + String(J1stepsToDo[sol]) + " J2stepsToDo: " + String(J2stepsToDo[sol]) + " J3stepsToDo: " + String(J3stepsToDo[sol]) + " J4stepsToDo: " + String(J4stepsToDo[sol]));
    if (J1stepsToDo[sol] > 0)
    {
      J1dir = 1;
    }
    if (J2stepsToDo[sol] > 0)
    {
      J2dir = 1;
    }
    if (J3stepsToDo[sol] > 0)
    {
      J3dir = 1;
    }
    if (J4stepsToDo[sol] > 0)
    {
      J4dir = 1;
    }
    J1stepsToDo[sol] = abs(J1stepsToDo[sol]);
    J2stepsToDo[sol] = abs(J2stepsToDo[sol]);
    J3stepsToDo[sol] = abs(J3stepsToDo[sol]);
    J4stepsToDo[sol] = abs(J4stepsToDo[sol]);

    DriveAxes(J1stepsToDo[sol], J2stepsToDo[sol], J3stepsToDo[sol], J4stepsToDo[sol], J1dir, J2dir, J3dir, J4dir, controlType, NORSpeed, ACCperc, DCCperc, TotTime, debug);

    TeensyState = CLEARING_BUFFER1;
  }
  break;

  // ================== SEND ROBOT POSITION ==================
  case RP:
  {
    Serial.println("ACKRP A" + String(J1stepsMaster) + " B" + String(J2stepsMaster) + " C" + String(J3stepsMaster) + " D" + String(J4stepsMaster));
    TeensyState = CLEARING_BUFFER1;
  }

  // ================== CLEAR BUFFERS AND GO TO IDLE ==================
  case CLEARING_BUFFER1:
  {
    cmdbuff1 = "";
    processedData = "";
    shiftbuffarray();
    TeensyState = IDLE;
  }
  break;
  }
}
