import processing.serial.*;
import controlP5.*;
import static processing.core.PApplet.*;


ControlP5 cp5;
Serial scaraSerial;
Serial nanoSerial;

PImage logoUAI;

String SCARAport = "";
String NANOport = "";
Textarea cons, programArea;
Println console;
int  scaraConnected = 0;
int nanoConnected = 0;
String receivedDataScara = "";
String receivedDataNano = "";
String processedDataScara = "";
String processedDataNano = "";
String folderPath = "";
String filePath = "";
boolean showCoordinateSys = false;
DropdownList ddlCotrolMode, ddlCotrolModeInverse ;
PrintWriter openedFile;
CheckBox enableCheckbox, homeCheckbox;
boolean toggleAirGripper= false;
boolean toggleAirSource = false;
boolean toggleFan = false;
int J1stepsMaster=0;
int J2stepsMaster=0;
int J3stepsMaster=0;
int J4stepsMaster=0;
int modelMode = 0;
int previousModelMode = 0;
boolean programRunning = false;
boolean ACKreceived = false;
boolean firstLine = false;
int lineNumber = 0;
String[] programLines;
RadioButton solutionButtons;
boolean inverseModel = false;
boolean forwardLinked = false;

float previousJ1Val=0;
float previousJ2Val=0;
float previousJ3Val=0;
float previousJ4Val=0;

final float J1stepsPerDeg = 55.55555555555556;
final float J2stepsPerDeg = 55.55555555555556;
final float J3stepsPerMM  = 78.74015748031496;
final float J4stepsPerDeg = 46.29629629629630;

float J1LimPos =125.0;
float J1LimNeg =-116.0;
float J2LimPos =120.0;
float J2LimNeg =-129.0;
float J3LimUpper= 0.0;
float J3LimLower =170.0;
float J4LimPos =120.0;
float J4LimNeg =-190.0;

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

float fw_xMaster=0;
float fw_yMaster=0;
float fw_zMaster=0;
float fw_fiMaster=0;

float[] inv_J1deg = {0.0, 0.0}; // left and right solution
float[] inv_J2deg = {0.0, 0.0};
float inv_J3distance = 0.0;
float[] inv_J4deg = {0.0, 0.0};

int leftHandedPossible = 0;
int rightHandedPossible = 0;

int i = 570;
int j = -200;


void setup() {
  fullScreen(2);
  //size(1920, 1080);
  logoUAI = loadImage("UAIlogoVert.png");
  cp5 = new ControlP5(this);
  PFont pfont = createFont("Arial", 25, true);
  ControlFont font = new ControlFont(pfont, 20);
  ControlFont font2 = new ControlFont(pfont, 18);
  cons = cp5.addTextarea("txt")
    .setPosition(50, 750)
    .setSize(810, 200)
    .setFont(createFont("Arial", 18))
    .setLineHeight(24)
    .setColor(color(0))
    .setColorBackground(color(0, 100))
    .setColorForeground(color(255, 100));
  ;
  console = cp5.addConsole(cons);

  cp5.addTextfield("SCARAport")
    .setPosition(50, 690)
    .setSize(90, 40)
    .setFont(font)
    .setAutoClear(false)
    .setColor(color(0))
    .setCaptionLabel("")
    .setColorBackground(color(0, 100))
    .setColorForeground(color(255, 1))
    .setColorActive(color(0, 0))
    .setText("COM")
    ;

  cp5.addTextfield("NANOport")
    .setPosition(150, 690)
    .setSize(90, 40)
    .setCaptionLabel("")
    .setFont(font)
    .setAutoClear(false)
    .setColor(color(0))
    .setColorBackground(color(0, 100))
    .setColorForeground(color(255, 1))
    .setColorActive(color(0, 0))
    .setText("COM")
    ;
  cp5.addButton("setPorts")
    .setPosition(260, 690)
    .setSize(135, 40)
    .setFont(font)
    .setCaptionLabel("set ports")
    .setColorBackground(color(0, 100))
    .setColorActive(color(20, 85))
    .setColorForeground(color(20, 70))
    .setColorCaptionLabel(255)
    ;
  cp5.addButton("refreshPorts")
    .setPosition(410, 690)
    .setSize(135, 40)
    .setFont(font)
    .setCaptionLabel("scan ports")
    .setColorBackground(color(0, 100))
    .setColorActive(color(0, 85))
    .setColorForeground(color(50, 70))
    .setColorCaptionLabel(255)
    ;
  cp5.addButton("connectPorts")
    .setPosition(560, 690)
    .setSize(135, 40)
    .setFont(font)
    .setCaptionLabel("Connect")
    .setColorBackground(color(50, 255, 50, 255))
    .setColorActive(color(10, 255, 10, 255))
    .setColorForeground(color(100, 255, 100, 255))
    .setColorCaptionLabel(255)
    ;
  cp5.addButton("disconnectPorts")
    .setPosition(710, 690)
    .setSize(150, 40)
    .setFont(font)
    .setCaptionLabel("Disconnect")
    .setColorBackground(color(255, 50, 50, 255))
    .setColorActive(color(255, 10, 10, 255))
    .setColorForeground(color(255, 100, 100, 255))
    .setColorCaptionLabel(255)
    ;


  //console control buttons

  cp5.addButton("pauseCons")
    .setPosition(870, 750)
    .setSize(100, 40)
    .setFont(font)
    .setCaptionLabel("pause")
    .setColorBackground(color(0, 100))
    .setColorActive(color(20, 85))
    .setColorForeground(color(20, 70))
    .setColorCaptionLabel(255)
    ;
  cp5.addButton("playCons")
    .setPosition(870, 800)
    .setSize(100, 40)
    .setFont(font)
    .setCaptionLabel("play")
    .setColorBackground(color(0, 100))
    .setColorActive(color(20, 85))
    .setColorForeground(color(20, 70))
    .setColorCaptionLabel(255)
    ;
  cp5.addButton("clearCons")
    .setPosition(870, 850)
    .setSize(100, 40)
    .setFont(font)
    .setCaptionLabel("clear")
    .setColorBackground(color(0, 100))
    .setColorActive(color(20, 85))
    .setColorForeground(color(20, 70))
    .setColorCaptionLabel(255)
    ;

  //enable motors
  enableCheckbox = cp5.addCheckBox("enableCheckbox")
    .setPosition(60, 610)
    .setColorBackground(color(20, 1))
    .setColorActive(color(0, 100))
    .setColorForeground(color(10, 150))
    .setSize(24, 24)
    .setItemsPerRow(4)
    .setSpacingColumn(10)
    .addItem("J1ena", 1)
    .addItem("J2ena", 1)
    .addItem("J3ena", 1)
    .addItem("J4ena", 1)
    .hideLabels();
  ;
  cp5.addButton("enaSend")
    .setPosition(240, 565)
    .setSize(80, 30)
    .setFont(font)
    .setCaptionLabel("SEND")
    .setColorBackground(color(0, 100))
    .setColorActive(color(20, 85))
    .setColorForeground(color(20, 70))
    .setColorCaptionLabel(255)
    ;
  cp5.addButton("enaTeach")
    .setPosition(240, 565+35)
    .setSize(80, 30)
    .setFont(font)
    .setCaptionLabel("Teach")
    .setColorBackground(color(0, 100))
    .setColorActive(color(20, 85))
    .setColorForeground(color(20, 70))
    .setColorCaptionLabel(255)
    ;
  //home axes checkbox
  homeCheckbox = cp5.addCheckBox("homeCheckbox")
    .setPosition(60, 440)
    .setColorBackground(color(20, 1))
    .setColorActive(color(0, 100))
    .setColorForeground(color(10, 150))
    .setSize(24, 24)
    .setItemsPerRow(4)
    .setSpacingColumn(10)
    .addItem("J1home", 1)
    .addItem("J2home", 1)
    .addItem("J3home", 1)
    .addItem("J4home", 1)
    .hideLabels();
  ;

  cp5.addButton("homeSend")
    .setPosition(240, 395)
    .setSize(80, 30)
    .setFont(font)
    .setCaptionLabel("SEND")
    .setColorBackground(color(0, 100))
    .setColorActive(color(20, 85))
    .setColorForeground(color(20, 70))
    .setColorCaptionLabel(255)
    ;
  cp5.addButton("homeTeach")
    .setPosition(240, 395+35)
    .setSize(80, 30)
    .setFont(font)
    .setCaptionLabel("Teach")
    .setColorBackground(color(0, 100))
    .setColorActive(color(20, 85))
    .setColorForeground(color(20, 70))
    .setColorCaptionLabel(255)
    ;
  cp5.addTextfield("homeSpeed")
    .setPosition(140, 475)
    .setSize(50, 40)
    .setFont(font)
    .setAutoClear(false)
    .setColor(color(0))
    .setCaptionLabel("")
    .setColorBackground(color(0, 100))
    .setColorForeground(color(255, 1))
    .setColorActive(color(0, 0))
    .setText("300")
    ;
  cp5.addButton("robotPosSend")
    .setPosition(240, 115-38)
    .setSize(80, 30)
    .setFont(font)
    .setCaptionLabel("SEND")
    .setColorBackground(color(0, 100))
    .setColorActive(color(20, 85))
    .setColorForeground(color(20, 70))
    .setColorCaptionLabel(255)
    ;
  cp5.addButton("robotPosTeach")
    .setPosition(240, 115)
    .setSize(80, 30)
    .setFont(font)
    .setCaptionLabel("Teach")
    .setColorBackground(color(0, 100))
    .setColorActive(color(20, 85))
    .setColorForeground(color(20, 70))
    .setColorCaptionLabel(255)
    ;
  cp5.addButton("delaySend")
    .setPosition(240, 185)
    .setSize(80, 30)
    .setFont(font)
    .setCaptionLabel("SEND")
    .setColorBackground(color(0, 100))
    .setColorActive(color(20, 85))
    .setColorForeground(color(20, 70))
    .setColorCaptionLabel(255)
    ;
  cp5.addButton("delayTeach")
    .setPosition(240, 185+35)
    .setSize(80, 30)
    .setFont(font)
    .setCaptionLabel("Teach")
    .setColorBackground(color(0, 100))
    .setColorActive(color(20, 85))
    .setColorForeground(color(20, 70))
    .setColorCaptionLabel(255)
    ;
  cp5.addTextfield("delayTime")
    .setPosition(120, 220)
    .setSize(70, 40)
    .setFont(font)
    .setAutoClear(false)
    .setColor(color(0))
    .setCaptionLabel("")
    .setColorBackground(color(0, 100))
    .setColorForeground(color(255, 1))
    .setColorActive(color(0, 0))
    .setText("2.0")
    ;
  cp5.addButton("testEndstopsSend")
    .setPosition(240, 305)
    .setSize(80, 30)
    .setFont(font)
    .setCaptionLabel("SEND")
    .setColorBackground(color(0, 100))
    .setColorActive(color(20, 85))
    .setColorForeground(color(20, 70))
    .setColorCaptionLabel(255)
    ;
  cp5.addToggle("toggleAirSource")
    .setPosition(410, 615)
    .setSize(60, 25)
    .setValue(true)
    .setMode(ControlP5.SWITCH)
    .setLabelVisible(false)
    .setColorBackground(color(0, 60))
    .setColorActive(color(100, 255))

    ;
  cp5.addToggle("toggleAirGripper")
    .setPosition(585, 615)
    .setSize(60, 25)
    .setValue(true)
    .setMode(ControlP5.SWITCH)
    .setLabelVisible(false)
    .setColorBackground(color(0, 60))
    .setColorActive(color(100, 255))
    ;
  cp5.addButton("pneuSend")
    .setPosition(740+75, 565)
    .setSize(80, 30)
    .setFont(font)
    .setCaptionLabel("SEND")
    .setColorBackground(color(0, 100))
    .setColorActive(color(20, 85))
    .setColorForeground(color(20, 70))
    .setColorCaptionLabel(255)
    ;
  cp5.addButton("pneuTeach")
    .setPosition(740+75, 565+35)
    .setSize(80, 30)
    .setFont(font)
    .setCaptionLabel("Teach")
    .setColorBackground(color(0, 100))
    .setColorActive(color(20, 85))
    .setColorForeground(color(20, 70))
    .setColorCaptionLabel(255)
    ;

  cp5.addSlider("sliderForwardJ1")
    .setPosition(400, 370)
    .setSize(280, 30)
    .setRange(J1LimNeg, J1LimPos)
    .setLabelVisible(false)
    .setColorBackground(color(0, 60))
    .setColorActive(color(100, 255))
    .setColorForeground(color(20, 70))
    .setSliderMode(Slider.FLEXIBLE)
    ;
  cp5.addSlider("sliderForwardJ2")
    .setPosition(400, 410)
    .setSize(280, 30)
    .setRange(J2LimNeg, J2LimPos)
    .setLabelVisible(false)
    .setColorBackground(color(0, 60))
    .setColorActive(color(100, 255))
    .setColorForeground(color(20, 70))
    .setSliderMode(Slider.FLEXIBLE)
    ;
  cp5.addSlider("sliderForwardJ3")
    .setPosition(400, 450)
    .setSize(280, 30)
    .setRange(J3LimUpper, J3LimLower)
    .setLabelVisible(false)
    .setColorBackground(color(0, 60))
    .setColorActive(color(100, 255))
    .setColorForeground(color(20, 70))
    .setSliderMode(Slider.FLEXIBLE)
    ;
  cp5.addSlider("sliderForwardJ4")
    .setPosition(400, 490)
    .setSize(280, 30)
    .setRange(J4LimNeg, J4LimPos)
    .setLabelVisible(false)
    .setColorBackground(color(0, 60))
    .setColorActive(color(100, 255))
    .setColorForeground(color(20, 70))
    .setSliderMode(Slider.FLEXIBLE)
    ;
  cp5.addTextfield("valueForwardJ1")
    .setPosition(690, 367)
    .setSize(45, 36)
    .setFont(font)
    .setAutoClear(false)
    .setColor(color(0))
    .setCaptionLabel("")
    .setColorBackground(color(0, 100))
    .setColorForeground(color(255, 1))
    .setColorActive(color(0, 0))
    .setText("0")
    ;
  cp5.addTextfield("valueForwardJ2")
    .setPosition(690, 407)
    .setSize(45, 36)
    .setFont(font)
    .setAutoClear(false)
    .setColor(color(0))
    .setCaptionLabel("")
    .setColorBackground(color(0, 100))
    .setColorForeground(color(255, 1))
    .setColorActive(color(0, 0))
    .setText("0")
    ;
  cp5.addTextfield("valueForwardJ3")
    .setPosition(690, 447)
    .setSize(45, 36)
    .setFont(font)
    .setAutoClear(false)
    .setColor(color(0))
    .setCaptionLabel("")
    .setColorBackground(color(0, 100))
    .setColorForeground(color(255, 1))
    .setColorActive(color(0, 0))
    .setText("0")
    ;
  cp5.addTextfield("valueForwardJ4")
    .setPosition(690, 487)
    .setSize(45, 36)
    .setFont(font)
    .setAutoClear(false)
    .setColor(color(0))
    .setCaptionLabel("")
    .setColorBackground(color(0, 100))
    .setColorForeground(color(255, 1))
    .setColorActive(color(0, 0))
    .setText("0")
    ;
  cp5.addTextfield("valueNorSpeed")
    .setPosition(590, 290)
    .setSize(55, 36)
    .setFont(font)
    .setAutoClear(false)
    .setColor(color(0))
    .setCaptionLabel("")
    .setColorBackground(color(0, 100))
    .setColorForeground(color(255, 1))
    .setColorActive(color(0, 0))
    .setText("0")
    ;
  cp5.addTextfield("valueAccPerc")
    .setPosition(430, 290)
    .setSize(45, 36)
    .setFont(font)
    .setAutoClear(false)
    .setColor(color(0))
    .setCaptionLabel("")
    .setColorBackground(color(0, 100))
    .setColorForeground(color(255, 1))
    .setColorActive(color(0, 0))
    .setText("0")
    ;
  cp5.addTextfield("valueDccPerc")
    .setPosition(430, 330)
    .setSize(45, 36)
    .setFont(font)
    .setAutoClear(false)
    .setColor(color(0))
    .setCaptionLabel("")
    .setColorBackground(color(0, 100))
    .setColorForeground(color(255, 1))
    .setColorActive(color(0, 0))
    .setText("0")
    ;
  cp5.addTextfield("valueTotTime")
    .setPosition(795, 290)
    .setSize(55, 36)
    .setFont(font)
    .setAutoClear(false)
    .setColor(color(0))
    .setCaptionLabel("")
    .setColorBackground(color(0, 100))
    .setColorForeground(color(255, 1))
    .setColorActive(color(0, 0))
    .setText("0")
    ;

  ddlCotrolMode = cp5.addDropdownList("controlModeForward")
    .setPosition(545, 335)
    .setOpen(false)
    .setSize(100, 120)
    .setFont(font)
    .setColorLabel(color(0))
    .setColorValueLabel(color(0))
    .setCaptionLabel("Mode")
    .setBackgroundColor(color(190))
    .setItemHeight(30)
    .setBarHeight(30)
    .addItem("auto ", "a")
    .addItem("speed ", "s")
    .addItem("time ", "t")
    .setColorBackground(color(20, 70))
    .setColorForeground(color(20, 70))
    .setColorActive(color(20, 70))
    ;
  cp5.addButton("forwardSend")
    .setPosition(815, 485-80)
    .setSize(80, 30)
    .setFont(font)
    .setCaptionLabel("SEND")
    .setColorBackground(color(0, 100))
    .setColorActive(color(20, 85))
    .setColorForeground(color(20, 70))
    .setColorCaptionLabel(255)
    ;
  cp5.addButton("forwardTeach")
    .setPosition(815, 485)
    .setSize(80, 30)
    .setFont(font)
    .setCaptionLabel("Teach")
    .setColorBackground(color(0, 100))
    .setColorActive(color(20, 85))
    .setColorForeground(color(20, 70))
    .setColorCaptionLabel(255)
    ;
     cp5.addButton("forwardLink")
    .setPosition(815, 485-135)
    .setSize(80, 30)
    .setFont(font)
    .setCaptionLabel("Link")
    .setColorBackground(color(0, 100))
    .setColorActive(color(20, 85))
    .setColorForeground(color(20, 70))
    .setColorCaptionLabel(255)
    ;
  cp5.addButton("forwardCalculate")
    .setPosition(815, 445)
    .setSize(80, 30)
    .setFont(font)
    .setCaptionLabel("CALC")
    .setColorBackground(color(0, 100))
    .setColorActive(color(20, 85))
    .setColorForeground(color(20, 70))
    .setColorCaptionLabel(255)
    ;
  cp5.addTextfield("terminal")
    .setPosition(50, 960)
    .setSize(810, 36)
    .setFont(font)
    .setAutoClear(false)
    .setColor(color(0))
    .setCaptionLabel("")
    .setColorBackground(color(#CDCDCD))
    .setColorForeground(color(255, 1))
    .setColorActive(color(0, 0))
    .setText("")
    ;
  cp5.addButton("terminalSend")
    .setPosition(870, 958)
    .setSize(100, 40)
    .setFont(font)
    .setCaptionLabel("SEND")
    .setColorBackground(color(0, 100))
    .setColorActive(color(20, 85))
    .setColorForeground(color(20, 70))
    .setColorCaptionLabel(255)
    ;
  cp5.addButton("terminalTeach")
    .setPosition(870, 958-45)
    .setSize(100, 40)
    .setFont(font)
    .setCaptionLabel("Teach")
    .setColorBackground(color(0, 100))
    .setColorActive(color(20, 85))
    .setColorForeground(color(20, 70))
    .setColorCaptionLabel(255)
    ;


  cp5.addTextfield("valueAccPercInverse")
    .setPosition(430+580, 90)
    .setSize(45, 36)
    .setFont(font)
    .setAutoClear(false)
    .setColor(color(0))
    .setCaptionLabel("")
    .setColorBackground(color(0, 100))
    .setColorForeground(color(255, 1))
    .setColorActive(color(0, 0))
    .setText("0")
    ;
  cp5.addTextfield("valueDccPercInverse")
    .setPosition(430+580, 130)
    .setSize(45, 36)
    .setFont(font)
    .setAutoClear(false)
    .setColor(color(0))
    .setCaptionLabel("")
    .setColorBackground(color(0, 100))
    .setColorForeground(color(255, 1))
    .setColorActive(color(0, 0))
    .setText("0")
    ;
  cp5.addTextfield("valueNorSpeedInverse")
    .setPosition(590+580, 90)
    .setSize(45, 36)
    .setFont(font)
    .setAutoClear(false)
    .setColor(color(0))
    .setCaptionLabel("")
    .setColorBackground(color(0, 100))
    .setColorForeground(color(255, 1))
    .setColorActive(color(0, 0))
    .setText("0")
    ;
  cp5.addTextfield("valueTotTimeInverse")
    .setPosition(795+580, 90)
    .setSize(55, 36)
    .setFont(font)
    .setAutoClear(false)
    .setColor(color(0))
    .setCaptionLabel("")
    .setColorBackground(color(0, 100))
    .setColorForeground(color(255, 1))
    .setColorActive(color(0, 0))
    .setText("0")
    ;
  cp5.addTextfield("xInverse")
    .setPosition(980, 185)
    .setSize(75+20, 36)
    .setFont(font2)
    .setAutoClear(false)
    .setColor(color(0))
    .setCaptionLabel("")
    .setColorBackground(color(0, 100))
    .setColorForeground(color(255, 1))
    .setColorActive(color(0, 0))
    .setText("0")
    .align(1000,1000,10000,10000) 
      
    ;
  cp5.addTextfield("yInverse")
    .setPosition(980, 225)
    .setSize(75+20, 36)
    .setFont(font2)
    .setAutoClear(false)
    .setColor(color(0))
    .setCaptionLabel("")
    .setColorBackground(color(0, 100))
    .setColorForeground(color(255, 1))
    .setColorActive(color(0, 0))
    .setText("0")
    ;
  cp5.addTextfield("zInverse")
    .setPosition(980, 265)
    .setSize(75+20, 36)
    .setFont(font2)
    .setAutoClear(false)
    .setColor(color(0))
    .setCaptionLabel("")
    .setColorBackground(color(0, 100))
    .setColorForeground(color(255, 1))
    .setColorActive(color(0, 0))
    .setText("0")
    ;
  cp5.addTextfield("fiInverse")
    .setPosition(980, 305)
    .setSize(75+20, 36)
    .setFont(font2)
    .setAutoClear(false)
    .setColor(color(0))
    .setCaptionLabel("")
    .setColorBackground(color(0, 100))
    .setColorForeground(color(255, 1))
    .setColorActive(color(0, 0))
    .setText("0")
    ;

  ddlCotrolModeInverse = cp5.addDropdownList("controlModeInverse")
    .setPosition(545+580, 135)
    .setOpen(false)
    .setSize(100, 120)
    .setFont(font)
    .setColorLabel(color(0))
    .setColorValueLabel(color(0))
    .setCaptionLabel("Mode")
    .setBackgroundColor(color(190))
    .setItemHeight(30)
    .setBarHeight(30)
    .addItem("auto ", "a")
    .addItem("speed ", "s")
    .addItem("time ", "t")
    .setColorBackground(color(20, 70))
    .setColorForeground(color(20, 70))
    .setColorActive(color(20, 70))
    ;
  cp5.addButton("inverseSend")
    .setPosition(815+720, 285+50)
    .setSize(80, 30)
    .setFont(font)
    .setCaptionLabel("SEND")
    .setColorBackground(color(0, 100))
    .setColorActive(color(20, 85))
    .setColorForeground(color(20, 70))
    .setColorCaptionLabel(255)
    ;
  cp5.addButton("inverseTeach")
    .setPosition(815+720, 255)
    .setSize(80, 30)
    .setFont(font)
    .setCaptionLabel("Teach")
    .setColorBackground(color(0, 100))
    .setColorActive(color(20, 85))
    .setColorForeground(color(20, 70))
    .setColorCaptionLabel(255)
    ;
  cp5.addButton("inverseCalculate")
    .setPosition(815+720, 245+50)
    .setSize(80, 30)
    .setFont(font)
    .setCaptionLabel("CALC")
    .setColorBackground(color(0, 100))
    .setColorActive(color(20, 85))
    .setColorForeground(color(20, 70))
    .setColorCaptionLabel(255)
    ;
  //solutionCheckbox = cp5.addCheckBox("solutionCheckbox")
  //  .setPosition(1560, 133)
  //  .setColorBackground(color(20, 1))
  //  .setColorActive(color(0, 100))
  //  .setColorForeground(color(10, 150))
  //  .setSize(24, 24)
  //  .setItemsPerRow(1)
  //  .setSpacingColumn(10)
  //  .setSpacingRow(10)
  //  .addItem("Left", 1)
  //  .addItem("Right", 1)
  //  .addItem("Both", 1)
  //  .hideLabels()
  //  .setNoneSelectedAllowed(false)
  //;
  solutionButtons = cp5.addRadioButton("solutionRadioButtons")
    .setPosition(1560, 133)
    .setSize(24, 24)
    .setColorBackground(color(20, 1))
    .setColorForeground(color(10, 150))
    .setColorActive(color(0, 100))
    .setColorLabel(color(255))
    .setItemsPerRow(1)
    .setSpacingColumn(10)
    .setSpacingRow(10)
    .addItem("Left", 1)
    .addItem("Right", 2)
    .addItem("Both", 3)
    .hideLabels()
    .setNoneSelectedAllowed(false)
    .activate(0)
    ;



  programArea = cp5.addTextarea("programAreaTxt")
    .setPosition(1035, 750-57)
    .setSize(600, 200+57)
    .setFont(createFont("Arial", 18))
    .setLineHeight(24)
    .setColor(color(0))
    .setColorBackground(color(0, 100))
    .setColorForeground(color(255, 100))
    .scroll(1)
    ;
  cp5.addButton("openFolder")
    .setPosition(1670, 90)
    .setSize(190, 30)
    .setFont(font)
    .setCaptionLabel("Choose folder")
    .setColorBackground(color(0, 100))
    .setColorActive(color(20, 85))
    .setColorForeground(color(20, 70))
    .setColorCaptionLabel(255)
    ;

  cp5.addTextfield("fileName")
    .setPosition(1750, 125)
    .setSize(110, 30)
    .setFont(font)
    .setAutoClear(false)
    .setCaptionLabel("")
    .setColor(color(0))
    .setColorBackground(color(#CDCDCD))
    .setColorForeground(color(255, 1))
    .setText("")
    ;
  cp5.addButton("createAndOpenFile")
    .setPosition(1670, 160)
    .setSize(190, 30)
    .setFont(font)
    .setCaptionLabel("Create & Open")
    .setColorBackground(color(0, 100))
    .setColorActive(color(20, 85))
    .setColorForeground(color(20, 70))
    .setColorCaptionLabel(255)
    ;
  cp5.addButton("OpenExistingFile")
    .setPosition(1670, 210)
    .setSize(190, 30)
    .setFont(font)
    .setCaptionLabel("Open file")
    .setColorBackground(color(0, 100))
    .setColorActive(color(20, 85))
    .setColorForeground(color(20, 70))
    .setColorCaptionLabel(255)
    ;
  cp5.addButton("closeFile")
    .setPosition(1670, 340-90)
    .setSize(190, 30)
    .setFont(font)
    .setCaptionLabel("Close File")
    .setColorBackground(color(0, 100))
    .setColorActive(color(20, 85))
    .setColorForeground(color(20, 70))
    .setColorCaptionLabel(255)
    ;
  cp5.addButton("updateProgramTextArea")
    .setPosition(1035, 965)
    .setSize(110, 30)
    .setFont(font)
    .setCaptionLabel("Refresh")
    .setColorBackground(color(0, 100))
    .setColorActive(color(20, 85))
    .setColorForeground(color(20, 70))
    .setColorCaptionLabel(255)
    ;
  cp5.addButton("readProgramToString")
    .setPosition(1150, 965)
    .setSize(70, 30)
    .setFont(font)
    .setCaptionLabel("Read")
    .setColorBackground(color(0, 100))
    .setColorActive(color(20, 85))
    .setColorForeground(color(20, 70))
    .setColorCaptionLabel(255)
    ;
  cp5.addButton("saveProgramToFile")
    .setPosition(1225, 965)
    .setSize(70, 30)
    .setFont(font)
    .setCaptionLabel("save")
    .setColorBackground(color(0, 100))
    .setColorActive(color(20, 85))
    .setColorForeground(color(20, 70))
    .setColorCaptionLabel(255)
    ;
  cp5.addButton("runProgram")
    .setPosition(1645, 700)
    .setSize(80, 50)
    .setFont(font)
    .setCaptionLabel("RUN")
    .setColorBackground(color(50, 255, 50, 255))
    .setColorActive(color(10, 255, 10, 255))
    .setColorForeground(color(100, 255, 100, 255))
    .setColorCaptionLabel(255)
    ;

  cp5.addButton("pauseProgram")
    .setPosition(1645, 760)
    .setSize(80, 50)
    .setFont(font)
    .setCaptionLabel("PAUSE")
    .setColorBackground(color(255, 150, 5, 255))
    .setColorActive(color(255, 124, 0, 255))
    .setColorForeground(color(255, 180, 70, 255))
    .setColorCaptionLabel(255)
    ;
  cp5.addButton("stopProgram")
    .setPosition(1645, 820)
    .setSize(80, 50)
    .setFont(font)
    .setCaptionLabel("STOP")
    .setColorBackground(color(255, 50, 50, 255))
    .setColorActive(color(255, 10, 10, 255))
    .setColorForeground(color(255, 100, 100, 255))
    .setColorCaptionLabel(255)
    ;
  cp5.addButton("showCS")
    .setPosition(1660, 610-155)
    .setSize(120, 30)
    .setFont(font)
    .setCaptionLabel("show cs")
    .setColorBackground(color(0, 100))
    .setColorActive(color(20, 85))
    .setColorForeground(color(20, 70))
    .setColorCaptionLabel(255)
    ;
  cp5.addButton("forwardModel")
    .setPosition(1660, 610-155+80)
    .setSize(120, 30)
    .setFont(font)
    .setCaptionLabel("Forward")
    .setColorBackground(color(0, 100))
    .setColorActive(color(20, 85))
    .setColorForeground(color(20, 70))
    .setColorCaptionLabel(255)
    ;
  cp5.addButton("inverseModel")
    .setPosition(1660, 610-35)
    .setSize(120, 30)
    .setFont(font)
    .setCaptionLabel("Inverse")
    .setColorBackground(color(0, 100))
    .setColorActive(color(20, 85))
    .setColorForeground(color(20, 70))
    .setColorCaptionLabel(255)
    ;
  cp5.addButton("liveModel")
    .setPosition(1660, 610-155+40)
    .setSize(120, 30)
    .setFont(font)
    .setCaptionLabel("Live")
    .setColorBackground(color(0, 100))
    .setColorActive(color(20, 85))
    .setColorForeground(color(20, 70))
    .setColorCaptionLabel(255)
    ;
  cp5.addButton("inverseModelLeft")
    .setPosition(1660, 610)
    .setSize(80, 30)
    .setFont(font)
    .setCaptionLabel("Left")
    .setColorBackground(color(0, 100))
    .setColorActive(color(20, 85))
    .setColorForeground(color(20, 70))
    .setColorCaptionLabel(255)
    ;
  cp5.addButton("inverseModelRight")
    .setPosition(1750, 610)
    .setSize(80, 30)
    .setFont(font)
    .setCaptionLabel("Right")
    .setColorBackground(color(0, 100))
    .setColorActive(color(20, 85))
    .setColorForeground(color(20, 70))
    .setColorCaptionLabel(255)
    ;
      cp5.addButton("fanSend")
    .setPosition(930+860, 80+255+32-20)
    .setSize(80, 27)
    .setFont(font)
    .setCaptionLabel("SEND")
    .setColorBackground(color(0, 100))
    .setColorActive(color(20, 85))
    .setColorForeground(color(20, 70))
    .setColorCaptionLabel(255)
    ;
  cp5.addButton("fanTeach")
    .setPosition(930+860, 80+255-20)
    .setSize(80, 27)
    .setFont(font)
    .setCaptionLabel("Teach")
    .setColorBackground(color(0, 100))
    .setColorActive(color(20, 85))
    .setColorForeground(color(20, 70))
    .setColorCaptionLabel(255)
    ;
    cp5.addToggle("toggleFan")
    .setPosition(930+755, 80+262)
    .setSize(60, 25)
    .setValue(true)
    .setMode(ControlP5.SWITCH)
    .setLabelVisible(false)
    .setColorBackground(color(0, 60))
    .setColorActive(color(100, 255))
    ;
}


void draw() {
  background(#F2F2F2);
  textSize(16);
  fill(0);
  text("SCARA Controller V1.0 by Tomas Janousek", 1630, 1070);
  text("SCARA", 50, 745);
  text("NANO", 150, 745);
  textSize(40);
  text("Instructions", 90, 50);
  text("Robot position", 500, 50);
  text("Forward kinematics", 470, 265);
  text("Inverse kinematics", 1130, 50);
  text("File Manager", 1660, 50);
  text("Model", 1715, 420);
  textSize(24);
  text("Enable motors", 50, 575);
  text("Home axes", 50, 405);
  text("Pneumatics control", 372, 575);
  text("Wait for seconds", 50, 195);
  text("Fan",930+700+15+10, 80+255);
 
  text("J1", 59, 435);
  text("J2", 93, 435);
  text("J3", 127, 435);
  text("J4", 161, 435);
  text("Speed", 60, 500);
  text("steps/s", 200, 500);

  text("Time", 60, 245);
  text("sec", 200, 245);

  text("J1", 59, 605);
  text("J2", 93, 605);
  text("J3", 127, 605);
  text("J4", 161, 605);

  text("J1", 530, 105-10);
  text("J2", 620, 105-10);
  text("J3", 710, 105-10);
  text("J4", 800, 105-10);

  text("X", 540, 185);
  text("Y", 630, 185);
  text("Z", 720, 185);
  text("Fi", 805, 185);

  text("Solution", 1500, 115);
  text("Left", 1500, 155);
  text("Right", 1500, 155+34);
  text("Both", 1500, 155+2*34);


  text("X", 950, 210);
  text("Y", 950, 250);
  text("Z", 950, 290);
  text("Fi", 950, 330);
  text("mm", 950+110+20, 210);
  text("mm", 950+110+20, 250);
  text("mm", 950+110+20, 290);
  text("deg", 950+110+20, 330);


  text("J1", 370, 395);
  text("J2", 370, 435);
  text("J3", 370, 475);
  text("J4", 370, 515);
  text("Accel", 370, 315);
  text("Speed", 520, 315);
  text("Decel", 370, 355);
  text("%", 480, 315);
  text("steps/s", 650, 315);
  text("%", 480, 355);

  text("Name", 1675, 146);
  text("Time", 740, 315);
  text("sec", 855, 315);

  text("Time", 740+580, 115);
  text("sec", 855+580, 115);
  text("Accel", 370+580, 115);
  text("Speed", 520+580, 115);
  text("Decel", 370+580, 155);
  text("%", 480+580, 115);
  text("steps/s", 650+580, 115);
  text("%", 480+580, 155);

  text("deg", 740, 395);
  text("deg", 740, 435);
  text("mm", 740, 475);
  text("deg", 740, 515);

  text("Master step", 372, 155-27-10);
  text("Angles/Dist", 372, 190-27-10);
  text("Cartesian", 372, 190+18);
  text("air source", 390, 605);
  text("gripper control", 540, 605);
  text("int", 372, 635);
  text("ext", 480, 635);
  
    text("on", 930+725, 80+282);
  text("off", 930+820, 80+282);
  
  text("open", 650, 635);
  text("close", 530, 635);
  text("Test endstops", 50, 315);
  text("Get position", 50, 105-10);
  text("Wait for seconds", 50, 195);
  text("Wait for seconds", 50, 195);
  
  image(logoUAI, 1795, 690,80,310);

  textAlign(RIGHT);
  text(J1stepsMaster, 580, 160-37);
  text(J2stepsMaster, 670, 160-37);
  text(J3stepsMaster, 760, 160-37);
  text(J4stepsMaster, 850, 160-37);
  textSize(20);
  text(J1stepsMaster/J1stepsPerDeg, 580, 195-37);
  text(J2stepsMaster/J2stepsPerDeg, 670, 195-37);
  text(J3stepsMaster/J3stepsPerMM, 760, 195-37);
  text(J4stepsMaster/J4stepsPerDeg, 850, 195-37);

  noStroke();
  fill( color(255, 50, 50, 190));
  rect(505, 170+18, 80, 30, 7);
  fill( color(50, 255, 50, 190));
  rect(595, 170+18, 80, 30, 7);
  fill( color(50, 50, 255, 190));
  rect(685, 170+18, 80, 30, 7);
  fill(color(255, 150, 5, 190));
  rect(775, 170+18, 80, 30, 7);
  fill(0);
  stroke(color(1, 85));
  text(fw_xMaster, 580, 195+18);
  text(fw_yMaster, 670, 195+18);
  text(fw_zMaster, 760, 195+18);
  text(fw_fiMaster, 850, 195+18);


  textAlign(LEFT);
  noStroke();
  fill(color(0, 60));
  rect(505, 135-37, 80, 30, 7);
  rect(595, 135-37, 80, 30, 7);
  rect(685, 135-37, 80, 30, 7);
  rect(775, 135-37, 80, 30, 7);
  rect(505, 170-37, 80, 30, 7);
  rect(595, 170-37, 80, 30, 7);
  rect(685, 170-37, 80, 30, 7);
  rect(775, 170-37, 80, 30, 7);

  noFill();
  stroke(color(1, 85));
  strokeWeight(3);
  //console and terminal
  rect(40, 680, 960, 330, 15);
  rect(1025, 680, 860, 330, 15);

  //home axes and enable main frame
  rect(40, 380, 300, 150, 15);
  rect(40, 550, 300, 100, 15);
  //test limit switches mainframe
  rect(40, 290, 300, 70, 15);
  //delay
  rect(40, 170, 300, 100, 15);
  //robot position
  rect(40, 80-10, 300, 70+10, 15);
  //enable motors checkbox
  rect(59, 609, 24, 24, 3);
  rect(93, 609, 24, 24, 3);
  rect(127, 609, 24, 24, 3);
  rect(161, 609, 24, 24, 3);
  //home motors checkbox
  rect(59, 439, 24, 24, 3);
  rect(93, 439, 24, 24, 3);
  rect(127, 439, 24, 24, 3);
  rect(161, 439, 24, 24, 3);
  //solution checkbox
  rect(1560, 133, 24, 24, 3);
  rect(1560, 167, 24, 24, 3);
  rect(1560, 201, 24, 24, 3);

  //robot absolute position frame
  rect(360, 80-10, 550, 145+10, 15);
  //forward kinematics
  rect(360, 280, 550, 250, 15);
  // gripper
  rect(360, 550, 550, 100, 15);
  // inverse kin
  rect(930, 80-10, 700, 570+10, 15);
  rect(930+700+15, 80-10, 240, 300+10-90, 15);
  rect(930+700+15, 80-10+240, 240, 70, 15);
  rect(930+700+15, 80+300+60, 240, 210, 15);
  
  if (scaraConnected==1 && scaraSerial.available()>0) {
    while (true) {
      char received = scaraSerial.readChar();
      receivedDataScara = receivedDataScara + received;
      if (received == '\n') {
        receivedDataScara = receivedDataScara.trim();
        processedDataScara = receivedDataScara;
        println(receivedDataScara);
        receivedDataScara="";
        break;
      }
    }
  }
  if (nanoConnected==1 && nanoSerial.available()>0) {
    while (true) {
      char received = nanoSerial.readChar();
      receivedDataNano = receivedDataNano + received;
      if (received == '\n') {
        receivedDataNano = receivedDataNano.trim();
        processedDataNano = receivedDataNano;
        println(receivedDataNano);
        receivedDataNano="";
        break;
      }
    }
  }

  if (processedDataScara.indexOf("ACK") != -1 || processedDataNano.indexOf("ACK") != -1) {
    ACKreceived = true;
  }

  if (processedDataScara.indexOf("ACKRP") != -1) {
    String message = processedDataScara.substring(6);
    int indexJ1 = message.indexOf("A");
    int indexJ2 = message.indexOf("B");
    int indexJ3 = message.indexOf("C");
    int indexJ4 = message.indexOf("D");

    J1stepsMaster =  int(message.substring(indexJ1+1, indexJ2-1));
    J2stepsMaster =  int(message.substring(indexJ2+1, indexJ3-1));
    J3stepsMaster =  int(message.substring(indexJ3+1, indexJ4-1));
    J4stepsMaster =  int(message.substring(indexJ4+1));
    fw_xMaster = L1 * cos(radians(float(J1stepsMaster)/J1stepsPerDeg)) + L2 * cos(radians(float(J1stepsMaster)/J1stepsPerDeg) + radians(float(J2stepsMaster)/J2stepsPerDeg));
    fw_yMaster = L1 * sin(radians(float(J1stepsMaster)/J1stepsPerDeg)) + L2 * sin(radians(float(J1stepsMaster)/J1stepsPerDeg) + radians(float(J2stepsMaster)/J2stepsPerDeg));
    fw_zMaster = d1 - (float(J3stepsMaster)/J3stepsPerMM) - dcor;
    fw_fiMaster = (float(J1stepsMaster)/J1stepsPerDeg)+(float(J2stepsMaster)/J2stepsPerDeg)+(float(J4stepsMaster)/J4stepsPerDeg);
  }

  if (programRunning) {

    if (lineNumber < programLines.length) {
      if ( ACKreceived || firstLine) {
        firstLine = false;
        ACKreceived = false;
        if (sendLine()) {
          //println("line send succesfully");
          lineNumber++;
        }
      }
    } else {
      //println("program end");
      modelMode = previousModelMode;
      lineNumber = 0;
      programRunning = false;
    }
  }
  //void ForwardKinematicsSolver(float J1angle, float J2angle, float J3distance, float J4angle)
  //{
  //  float J1angleRad = radians(J1angle);
  //  float J2angleRad = radians(J2angle);
  //  float J4angleRad = radians(J4angle);
  //  fw_x = L1 * cos(J1angleRad) + L2 * cos(J1angleRad + J2angleRad);
  //  fw_y = L1 * sin(J1angleRad) + L2 * sin(J1angleRad + J2angleRad);
  //  fw_z = d1 - J3distance - dcor;
  //  fw_fi = degrees((J1angleRad + J2angleRad + J4angleRad));
  //}

  float modelJ1Angle=radians(0);
  float modelJ2Angle=radians(0);
  float modelJ3distance = 0;
  float modelJ4Angle = radians(0);

  cp5.getController("forwardModel").setColorBackground(color(0, 100));
  cp5.getController("forwardModel").setColorActive(color(20, 85));
  cp5.getController("forwardModel").setColorForeground(color(20, 70));

  cp5.getController("inverseModel").setColorBackground(color(0, 100));
  cp5.getController("inverseModel").setColorActive(color(20, 85));
  cp5.getController("inverseModel").setColorForeground(color(20, 70));

  cp5.getController("liveModel").setColorBackground(color(0, 100));
  cp5.getController("liveModel").setColorActive(color(20, 85));
  cp5.getController("liveModel").setColorForeground(color(20, 70));

  cp5.getController("inverseModelLeft").setColorBackground(color(0, 100));
  cp5.getController("inverseModelLeft").setColorActive(color(20, 85));
  cp5.getController("inverseModelLeft").setColorForeground(color(20, 70));

  cp5.getController("inverseModelRight").setColorBackground(color(0, 100));
  cp5.getController("inverseModelRight").setColorActive(color(20, 85));
  cp5.getController("inverseModelRight").setColorForeground(color(20, 70));



  if ( modelMode == 1 ) {
    cp5.getController("forwardModel").setColorBackground(color(50, 255, 50, 255));
    cp5.getController("forwardModel").setColorActive(color(10, 255, 10, 255));
    cp5.getController("forwardModel").setColorForeground(color(100, 255, 100, 255));

    modelJ1Angle = radians(float(cp5.get(Textfield.class, "valueForwardJ1").getText()));
    modelJ2Angle = radians(float(cp5.get(Textfield.class, "valueForwardJ2").getText()));
    modelJ3distance = float(cp5.get(Textfield.class, "valueForwardJ3").getText());
    modelJ4Angle = radians(float(cp5.get(Textfield.class, "valueForwardJ4").getText()));
  }
  if ( modelMode == 2 ) {
    cp5.getController("inverseModel").setColorBackground(color(50, 255, 50, 255));
    cp5.getController("inverseModel").setColorActive(color(10, 255, 10, 255));
    cp5.getController("inverseModel").setColorForeground(color(100, 255, 100, 255));
    if (!inverseModel) {
      cp5.getController("inverseModelLeft").setColorBackground(color(50, 255, 50, 255));
      cp5.getController("inverseModelLeft").setColorActive(color(10, 255, 10, 255));
      cp5.getController("inverseModelLeft").setColorForeground(color(100, 255, 100, 255));
      modelJ1Angle = radians(inv_J1deg[0]);
      modelJ2Angle = radians(inv_J2deg[0]);
      modelJ3distance = inv_J3distance;
      modelJ4Angle = radians(inv_J4deg[0]);
    }
    if (inverseModel) {
      cp5.getController("inverseModelRight").setColorBackground(color(50, 255, 50, 255));
      cp5.getController("inverseModelRight").setColorActive(color(10, 255, 10, 255));
      cp5.getController("inverseModelRight").setColorForeground(color(100, 255, 100, 255));
      modelJ1Angle = radians(inv_J1deg[1]);
      modelJ2Angle = radians(inv_J2deg[1]);
      modelJ3distance = inv_J3distance;
      modelJ4Angle = radians(inv_J4deg[1]);
    }
  }
  if ( modelMode == 3 ) {
    modelJ1Angle = radians(J1stepsMaster/J1stepsPerDeg);
    modelJ2Angle = radians(J2stepsMaster/J2stepsPerDeg);
    modelJ3distance = J3stepsMaster/J3stepsPerMM;
    modelJ4Angle = radians(J4stepsMaster/J4stepsPerDeg);

    cp5.getController("liveModel").setColorBackground(color(50, 255, 50, 255));
    cp5.getController("liveModel").setColorActive(color(10, 255, 10, 255));
    cp5.getController("liveModel").setColorForeground(color(100, 255, 100, 255));
  }

  //float modelJ1Angle=radians(90);
  //float modelJ2Angle=radians(-90);
  //float modelJ3 = 10;
  //float modelJ4Angle = radians(0);
  int xBase = 1250;
  int yBase = 500;

  float scl = 0.6;
  noStroke();
  fill(color(50, 255));
  rect(xBase, yBase, 130*scl, 185*scl, 20, 20, 10, 10);
  fill(#999999);
  circle(xBase+(130*scl)/2, yBase+65*scl, 130*scl);
  stroke(color(1, 85));
  line(xBase+(130*scl)/2, yBase+65*scl, xBase+(130*scl)/2-L1*scl * sin(modelJ1Angle), yBase+65*scl-L1*scl * cos(modelJ1Angle));

  if (showCoordinateSys) {
    fill(#FF1F1F);
    stroke(#FF1F1F);
    text("X", xBase + 90*scl, yBase -410*scl);
    line(xBase+(130*scl)/2, yBase+65*scl, xBase+(130*scl)/2, yBase+65*scl-500*scl);
    line(xBase+(130*scl)/2-10*scl, yBase+65*scl-500*scl+20*scl, xBase+(130*scl)/2, yBase+65*scl-500*scl);
    line(xBase+(130*scl)/2+10*scl, yBase+65*scl-500*scl+20*scl, xBase+(130*scl)/2, yBase+65*scl-500*scl);
    stroke(#0AFC08);
    fill(#0AFC08);
    text("Y", xBase -430*scl, yBase +110*scl);
    line(xBase+(130*scl)/2-500*scl, yBase+65*scl, xBase+(130*scl)/2, yBase+65*scl);
    line(xBase+(130*scl)/2-500*scl, yBase+65*scl, xBase+(130*scl)/2-500*scl+20*scl, yBase+65*scl+10*scl);
    line(xBase+(130*scl)/2-500*scl, yBase+65*scl, xBase+(130*scl)/2-500*scl+20*scl, yBase+65*scl-10*scl);
  }

  stroke(color(1, 85));
  fill(#CDCDCD);
  quad(xBase+(130*scl)/2-55*scl * cos(modelJ1Angle), yBase+65*scl+55*scl * sin(modelJ1Angle), xBase+(130*scl)/2+55*scl * cos(modelJ1Angle), yBase+65*scl-55*scl * sin(modelJ1Angle), xBase+(130*scl)/2-L1*scl * sin(modelJ1Angle)+55*scl * cos(modelJ1Angle), yBase+65*scl-L1*scl * cos(modelJ1Angle)-55*scl * sin(modelJ1Angle), xBase+(130*scl)/2-L1*scl * sin(modelJ1Angle)-55*scl * cos(modelJ1Angle), yBase+65*scl-L1*scl * cos(modelJ1Angle)+55*scl * sin(modelJ1Angle));
  circle(xBase+(130*scl)/2, yBase+65*scl, 110*scl);
  circle(xBase+(130*scl)/2-L1*scl * sin(modelJ1Angle), yBase+65*scl-L1*scl * cos(modelJ1Angle), 110*scl);
  fill(color(50, 255));
  noStroke();
  circle(xBase+(130*scl)/2-L1*scl * sin(modelJ1Angle), yBase+65*scl-L1*scl * cos(modelJ1Angle), 100*scl);
  circle(xBase+(130*scl)/2 -L1*scl * sin(modelJ1Angle) - L2*scl * sin(modelJ1Angle + modelJ2Angle),
    yBase+65*scl-L1*scl * cos(modelJ1Angle)- L2*scl * cos(modelJ1Angle + modelJ2Angle), 100*scl);


  quad(xBase+(130*scl)/2-L1*scl * sin(modelJ1Angle)-50*scl * cos(modelJ1Angle+ modelJ2Angle), yBase+65*scl-L1*scl * cos(modelJ1Angle)+50*scl * sin(modelJ1Angle+ modelJ2Angle),
    xBase+(130*scl)/2-L1*scl * sin(modelJ1Angle)+50*scl * cos(modelJ1Angle+ modelJ2Angle), yBase+65*scl-L1*scl * cos(modelJ1Angle)-50*scl * sin(modelJ1Angle+ modelJ2Angle),
    xBase+(130*scl)/2 -L1*scl * sin(modelJ1Angle) - L2*scl * sin(modelJ1Angle + modelJ2Angle)+50*scl * cos(modelJ1Angle+modelJ2Angle), yBase+65*scl-L1*scl * cos(modelJ1Angle)- L2*scl * cos(modelJ1Angle + modelJ2Angle)-50*scl * sin(modelJ1Angle+ modelJ2Angle),
    xBase+(130*scl)/2 -L1*scl * sin(modelJ1Angle) - L2*scl * sin(modelJ1Angle + modelJ2Angle)-50*scl * cos(modelJ1Angle+modelJ2Angle), yBase+65*scl-L1*scl * cos(modelJ1Angle)- L2*scl * cos(modelJ1Angle + modelJ2Angle)+50*scl * sin(modelJ1Angle+ modelJ2Angle)
    );

  fill(#595959);
  circle(xBase+(130*scl)/2 -L1*scl * sin(modelJ1Angle) - L2*scl * sin(modelJ1Angle + modelJ2Angle), yBase+65*scl-L1*scl * cos(modelJ1Angle)- L2*scl * cos(modelJ1Angle + modelJ2Angle), 60*scl);

  if (showCoordinateSys) {
    stroke(#FF1F1F);
    line(xBase+(130*scl)/2 -L1*scl * sin(modelJ1Angle) - L2*scl * sin(modelJ1Angle + modelJ2Angle), yBase+65*scl-L1*scl * cos(modelJ1Angle)- L2*scl * cos(modelJ1Angle + modelJ2Angle),
      xBase+(130*scl)/2 -L1*scl * sin(modelJ1Angle) - L2*scl * sin(modelJ1Angle + modelJ2Angle)-100*scl*sin(modelJ1Angle + modelJ2Angle+modelJ4Angle), yBase+65*scl-L1*scl * cos(modelJ1Angle)- L2*scl * cos(modelJ1Angle + modelJ2Angle)-100*scl*cos(modelJ1Angle + modelJ2Angle+modelJ4Angle));

    stroke(#0AFC08);
    line(xBase+(130*scl)/2 -L1*scl * sin(modelJ1Angle) - L2*scl * sin(modelJ1Angle + modelJ2Angle), yBase+65*scl-L1*scl * cos(modelJ1Angle)- L2*scl * cos(modelJ1Angle + modelJ2Angle),
      xBase+(130*scl)/2 -L1*scl * sin(modelJ1Angle) - L2*scl * sin(modelJ1Angle + modelJ2Angle)-100*scl*cos(modelJ1Angle + modelJ2Angle+modelJ4Angle), yBase+65*scl-L1*scl * cos(modelJ1Angle)- L2*scl * cos(modelJ1Angle + modelJ2Angle)+100*scl*sin(modelJ1Angle + modelJ2Angle+modelJ4Angle));
  }
  
  if(forwardLinked){
  float J1angleRad = radians(float(cp5.get(Textfield.class, "valueForwardJ1").getText()));
  float J2angleRad = radians(float(cp5.get(Textfield.class, "valueForwardJ2").getText()));
  float J3distance = float(cp5.get(Textfield.class, "valueForwardJ3").getText());
  float J4angleRad = radians(float(cp5.get(Textfield.class, "valueForwardJ4").getText()));
    
    if( J1angleRad != previousJ1Val || J2angleRad != previousJ2Val || J3distance != previousJ3Val || J4angleRad != previousJ4Val){
     float fw_x = L1 * cos(J1angleRad) + L2 * cos(J1angleRad + J2angleRad);
     float fw_y = L1 * sin(J1angleRad) + L2 * sin(J1angleRad + J2angleRad);
     float fw_z = d1 - J3distance - dcor;
     float fw_fi = degrees((J1angleRad + J2angleRad + J4angleRad));
     
      cp5.get(Textfield.class, "xInverse").setText(str(fw_x));
      cp5.get(Textfield.class, "yInverse").setText(str(fw_y));
      cp5.get(Textfield.class, "zInverse").setText(str(fw_z));
      cp5.get(Textfield.class, "fiInverse").setText(str(fw_fi));
     
     previousJ1Val= J1angleRad;
     previousJ2Val= J2angleRad;
     previousJ3Val= J3distance;
     previousJ4Val= J4angleRad;
     
     
    }

  }
  
  


  processedDataScara ="";
  processedDataNano ="";
}


public void setPorts() {
  SCARAport  = cp5.get(Textfield.class, "SCARAport").getText();
  NANOport  = cp5.get(Textfield.class, "NANOport").getText();

  println("Comunication ports set: SCARAport = " + SCARAport + ", NANOport = " + NANOport);
  println("____________________");
}
public void refreshPorts() {
  println("Available serial ports:");
  printArray(Serial.list());
  println("____________________");
}
public void clearCons() {
  console.clear();
}
public void playCons() {
  console.play();
}
public void pauseCons() {
  console.pause();
}
public void connectPorts() {

  SCARAport  = cp5.get(Textfield.class, "SCARAport").getText();
  NANOport  = cp5.get(Textfield.class, "NANOport").getText();


  if (SCARAport != "" && scaraConnected == 0) {
    try {
      scaraSerial = new Serial(this, SCARAport, 9600);
      cp5.getController("SCARAport").setColorBackground(color(50, 255, 50, 255));
      scaraConnected = 1;
    }
    catch(RuntimeException e) {
      scaraConnected = 0;
      cp5.getController("SCARAport").setColorBackground(color(0, 100));
      println(e);
      int index =  e.toString().indexOf("Port busy");
      println(index);
    }
  }

  if (NANOport != "" && nanoConnected == 0) {
    try {
      nanoSerial = new Serial(this, NANOport, 9600);
      cp5.getController("NANOport").setColorBackground(color(50, 255, 50, 255));
      nanoConnected = 1;
    }
    catch(RuntimeException e) {
      nanoConnected = 0;
      cp5.getController("NANOport").setColorBackground(color(0, 100));
      println(e);
      int index =  e.toString().indexOf("Port busy");
      println(index);
    }
  }
}
public void disconnectPorts() {
  if (scaraConnected == 1) {
    scaraSerial.stop();
    cp5.getController("SCARAport").setColorBackground(color(0, 100));
    scaraConnected = 0;
  }
  if (nanoConnected == 1) {
    nanoSerial.stop();
    cp5.getController("NANOport").setColorBackground(color(0, 100));
    nanoConnected = 0;
  }
}
public void enaSend() {
  int Vals[] = int(enableCheckbox.getArrayValue());
  String message = "EN A"+Vals[0]+" B"+Vals[1]+" C"+Vals[2]+" D"+Vals[3];
  enableCheckbox.deactivateAll();
  println(message);
  if (scaraConnected == 1) {
    scaraSerial.write(message+"\n");
  } else {
    println("SCARA controller disconnected");
  }
  println("____________________");
}

public void homeSend() {
  int Vals[] = int(homeCheckbox.getArrayValue());
  String speed = cp5.get(Textfield.class, "homeSpeed").getText();
  String message = "HS A"+Vals[0]+" B"+Vals[1]+" C"+Vals[2]+" D"+Vals[3]+" S"+speed;
  homeCheckbox.deactivateAll();
  println(message);
  if (scaraConnected == 1) {
    scaraSerial.write(message+"\n");
  } else {
    println("SCARA controller disconnected");
  }
  println("____________________");
}

public void robotPosSend() {
  String message = "RP ";
  println(message);
  if (scaraConnected == 1) {
    scaraSerial.write(message+"\n");
  } else {
    println("SCARA controller disconnected");
  }
  println("____________________");
}
public void testEndstopsSend() {
  String message = "TL ";
  println(message);
  if (scaraConnected == 1) {
    scaraSerial.write(message+"\n");
  } else {
    println("SCARA controller disconnected");
  }
  println("____________________");
}
public void delaySend() {
  String waitTime = cp5.get(Textfield.class, "delayTime").getText();
  String message = "DL " + waitTime ;
  println(message);
  if (scaraConnected == 1) {
    scaraSerial.write(message+"\n");
  } else {
    println("SCARA controller disconnected");
  }
  println("____________________");
}
public void pneuSend() {
  String nanoMessage1 = "";
  String nanoMessage2 = "";

  if (toggleAirSource) {
    nanoMessage1 = "P6Q0 S9T1 ";
  } else {
    nanoMessage1 = "P6Q1 S9T0 ";
  }

  if (toggleAirGripper) {
    nanoMessage2 = "U7V0";
  } else {
    nanoMessage2 = "U7V1";
  }
  String message ="RL "+ nanoMessage1+nanoMessage2  ;

  println(message);

  if (nanoConnected == 1) {
    nanoSerial.write(message + "\n");
  } else {
    println("NANO controller disconnected");
  }
  println("____________________");
}

public void sliderForwardJ1(float value) {
  cp5.get(Textfield.class, "valueForwardJ1").setText(str(round(value)));
}

public void sliderForwardJ2(float value) {
  cp5.get(Textfield.class, "valueForwardJ2").setText(str(round(value)));
}
public void sliderForwardJ3(float value) {

  cp5.get(Textfield.class, "valueForwardJ3").setText(str(round(value)));
}
public void sliderForwardJ4(float value) {
  cp5.get(Textfield.class, "valueForwardJ4").setText(str(round(value)));
}
public void valueForwardJ1(String text) {
  cp5.get(Slider.class, "sliderForwardJ1").setValue(float(text));
}
public void valueForwardJ2(String text) {
  cp5.get(Slider.class, "sliderForwardJ2").setValue(float(text));
}
public void valueForwardJ3(String text) {
  cp5.get(Slider.class, "sliderForwardJ3").setValue(float(text));
}
public void valueForwardJ4(String text) {
  cp5.get(Slider.class, "sliderForwardJ4").setValue(float(text));
}

public void forwardSend() {
  float J1angle = float(cp5.get(Textfield.class, "valueForwardJ1").getText());
  float J2angle = float(cp5.get(Textfield.class, "valueForwardJ2").getText());
  float J3distance = float(cp5.get(Textfield.class, "valueForwardJ3").getText());
  float J4angle = float(cp5.get(Textfield.class, "valueForwardJ4").getText());
  float accPerc = float(cp5.get(Textfield.class, "valueAccPerc").getText());
  float dccPerc = float(cp5.get(Textfield.class, "valueDccPerc").getText());
  float norSpeed = float(cp5.get(Textfield.class, "valueNorSpeed").getText());
  float totTime = float(cp5.get(Textfield.class, "valueTotTime").getText());
  String controlMode = "";
  int debug = 0;

  if (int(ddlCotrolMode.getValue()) == 0) {
    controlMode = "a";
  }
  if (int(ddlCotrolMode.getValue()) == 1) {
    controlMode = "s";
  }
  if (int(ddlCotrolMode.getValue()) == 2) {
    controlMode = "t";
  }


  if (J1angle !=J1angle) {
    J1angle = 0;
  }
  if (J2angle !=J2angle ) {
    J2angle  = 0;
  }
  if (J3distance !=J3distance) {
    J3distance = 0;
  }
  if (J4angle !=J4angle) {
    J4angle = 0;
  }
  if (accPerc !=accPerc) {
    accPerc = 0;
  }
  if (dccPerc !=dccPerc) {
    dccPerc = 0;
  }
  if (norSpeed !=norSpeed) {
    norSpeed = 0;
  }
  if (totTime !=totTime) {
    totTime = 0;
  }
  if (accPerc == 0 && controlMode == "s") {
    println("zero parameter error");
    println("____________________");
    return;
  }
  if (dccPerc == 0 && controlMode == "s" ) {
    println("zero parameter error");
    println("____________________");
    return;
  }
  if (norSpeed == 0 && controlMode == "s") {
    println("zero parameter error");
    println("____________________");
    return;
  }

  String message = "JJ A"+J1angle+" B"+J2angle+" C"+J3distance+" D"+J4angle+" E" + controlMode + " F" + norSpeed +" G" + accPerc +" H" + dccPerc + " I" + totTime + " J" + debug ;
  println(message);
  ForwardKinematicsSolver(J1angle, J2angle, J3distance, J4angle);
  println("x: " + fw_x + " y: " + fw_y + " z: " + fw_z + " fi: " + fw_fi);
  if ( checkLimitsForward(J1angle, J2angle, J3distance, J4angle)== 1 ) {
    println("Jog is out of robot axis limits");
    return;
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

  int AxisMostSteps = abs(J1stepsToDo);
  if (abs(J2stepsToDo) > AxisMostSteps)
  {
    AxisMostSteps = abs(J2stepsToDo);
  }
  if (abs(J3stepsToDo) > AxisMostSteps)
  {
    AxisMostSteps = abs(J3stepsToDo);
  }
  if (abs(J4stepsToDo) > AxisMostSteps)
  {
    AxisMostSteps = abs(J4stepsToDo);
  }

  if (J1stepsToDo == 0 && J2stepsToDo == 0 && J3stepsToDo == 0 && J4stepsToDo == 0)
  {
    println("Joints are already in the desired position");
  }
  println("J1stepsToDo: " + J1stepsToDo + " J2stepsToDo: " + J2stepsToDo + " J3stepsToDo: " + J3stepsToDo + " J4stepsToDo: " +J4stepsToDo );
  println("AxisMostSteps: " + AxisMostSteps);
  if (scaraConnected == 1) {
    scaraSerial.write(message+"\n");
  } else {
    println("SCARA controller disconnected");
  }
  println("____________________");
}
public void forwardCalculate() {
  float J1angle = float(cp5.get(Textfield.class, "valueForwardJ1").getText());
  float J2angle = float(cp5.get(Textfield.class, "valueForwardJ2").getText());
  float J3distance = float(cp5.get(Textfield.class, "valueForwardJ3").getText());
  float J4angle = float(cp5.get(Textfield.class, "valueForwardJ4").getText());
  float accPerc = float(cp5.get(Textfield.class, "valueAccPerc").getText());
  float dccPerc = float(cp5.get(Textfield.class, "valueDccPerc").getText());
  float norSpeed = float(cp5.get(Textfield.class, "valueNorSpeed").getText());
  float totTime = float(cp5.get(Textfield.class, "valueTotTime").getText());
  String controlMode = "";
  int debug = 0;

  if (int(ddlCotrolMode.getValue()) == 0) {
    controlMode = "a";
  }
  if (int(ddlCotrolMode.getValue()) == 1) {
    controlMode = "s";
  }
  if (int(ddlCotrolMode.getValue()) == 2) {
    controlMode = "t";
  }


  if (J1angle !=J1angle) {
    J1angle = 0;
  }
  if (J2angle !=J2angle ) {
    J2angle  = 0;
  }
  if (J3distance !=J3distance) {
    J3distance = 0;
  }
  if (J4angle !=J4angle) {
    J4angle = 0;
  }
  if (accPerc !=accPerc) {
    accPerc = 0;
  }
  if (dccPerc !=dccPerc) {
    dccPerc = 0;
  }
  if (norSpeed !=norSpeed) {
    norSpeed = 0;
  }
  if (totTime !=totTime) {
    totTime = 0;
  }

  String message = "JJ A"+J1angle+" B"+J2angle+" C"+J3distance+" D"+J4angle+" E" + controlMode + " F" + norSpeed +" G" + accPerc +" H" + dccPerc + " I" + totTime + " J" + debug ;
  println(message);
  ForwardKinematicsSolver(J1angle, J2angle, J3distance, J4angle);
  println("x: " + fw_x + " y: " + fw_y + " z: " + fw_z + " fi: " + fw_fi);

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

  int AxisMostSteps = abs(J1stepsToDo);
  if (abs(J2stepsToDo) > AxisMostSteps)
  {
    AxisMostSteps = abs(J2stepsToDo);
  }
  if (abs(J3stepsToDo) > AxisMostSteps)
  {
    AxisMostSteps = abs(J3stepsToDo);
  }
  if (abs(J4stepsToDo) > AxisMostSteps)
  {
    AxisMostSteps = abs(J4stepsToDo);
  }

  if (J1stepsToDo == 0 && J2stepsToDo == 0 && J3stepsToDo == 0 && J4stepsToDo == 0)
  {
    println("Joints are already in the desired position");
  }
  println("J1stepsToDo: " + J1stepsToDo + " J2stepsToDo: " + J2stepsToDo + " J3stepsToDo: " + J3stepsToDo + " J4stepsToDo: " +J4stepsToDo );
  println("AxisMostSteps: " + AxisMostSteps);
  println("____________________");
}

void ForwardKinematicsSolver(float J1angle, float J2angle, float J3distance, float J4angle)
{
  float J1angleRad = radians(J1angle);
  float J2angleRad = radians(J2angle);
  float J4angleRad = radians(J4angle);
  fw_x = L1 * cos(J1angleRad) + L2 * cos(J1angleRad + J2angleRad);
  fw_y = L1 * sin(J1angleRad) + L2 * sin(J1angleRad + J2angleRad);
  fw_z = d1 - J3distance - dcor;
  fw_fi = degrees((J1angleRad + J2angleRad + J4angleRad));
}

void InverseKinematicsSolver(float inv_x, float inv_y, float inv_z, float inv_fi)
{
  kinError = 0; // reset the error flag

  // calculate the angle of the second joint
  float inv_fiRad = radians(inv_fi);
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
  if (J1angleSolLeftRad > PI)
  {
    J1angleSolLeftRad = J1angleSolLeftRad - TWO_PI;
  }

  if (J1angleSolLeftRad < -PI)
  {
    J1angleSolLeftRad = TWO_PI + J1angleSolLeftRad;
  }
  // J1 angle right solution correction
  if (J1angleSolRightRad > PI)
  {
    J1angleSolRightRad = J1angleSolRightRad - TWO_PI;
  }

  if (J1angleSolRightRad < -PI)
  {
    J1angleSolRightRad = J1angleSolRightRad + TWO_PI;
  }
  // J3 angle left solution correction
  if (J4angleSolLeftRad < -PI)
  {
    J4angleSolLeftRad = J4angleSolLeftRad + TWO_PI;
  }
  if (J4angleSolLeftRad > PI)
  {
    J4angleSolLeftRad = J4angleSolLeftRad - TWO_PI;
  }
  // J3 angle right solution correction
  if (J4angleSolRightRad < -PI)
  {
    J4angleSolRightRad = J4angleSolRightRad + TWO_PI;
  }
  if (J4angleSolRightRad > PI)
  {
    J4angleSolRightRad = J4angleSolRightRad - TWO_PI;
  }
  if (J4angleSolRightRad < -TWO_PI)
  {
    J4angleSolRightRad = J4angleSolRightRad + TWO_PI;
  }


  // convert the angles to degrees
  inv_J1deg[0] = degrees(J1angleSolLeftRad);
  inv_J1deg[1] = degrees(J1angleSolRightRad);
  inv_J2deg[0] = degrees(J2angleSolLeftRad);
  inv_J2deg[1] = degrees(J2angleSolRightRad);
  inv_J3distance = d1 - inv_z - dcor; // entire calculation for z axis
  inv_J4deg[0] = degrees(J4angleSolLeftRad);
  inv_J4deg[1] = degrees(J4angleSolRightRad);
}


public void terminalSend() {
  String message = cp5.get(Textfield.class, "terminal").getText();
  String instruction = message.substring(0, 2);
  instruction = instruction.trim();
  println(message);

  if (instruction.equals("CS") || instruction.equals("RL")) {
    if (nanoConnected == 1) {
      nanoSerial.write(message+"\n");
    } else {
      println("NANO controller disconnected");
    }
    println("____________________");
  } else
  {
    if (scaraConnected == 1) {
      scaraSerial.write(message+"\n");
    } else {
      println("SCARA controller disconnected");
    }
    println("____________________");
  }

  cp5.get(Textfield.class, "terminal").clear();
}

void checkLimitsInverse()
{
  leftHandedPossible = 0;
  rightHandedPossible = 0;
  // check if the solution is possible
  if (((J1LimNeg < inv_J1deg[0]) && (inv_J1deg[0] < J1LimPos)) && ((J2LimNeg < inv_J2deg[0]) && (inv_J2deg[0] < J2LimPos)) && ((J4LimNeg < inv_J4deg[0]) && (J4LimPos > inv_J4deg[0])))
  {
    leftHandedPossible = 1;
  } else
  {
    println("left handed solution is over angle limits");
  }

  if (((J1LimNeg < inv_J1deg[1]) && (inv_J1deg[1] < J1LimPos)) && ((J2LimNeg < inv_J2deg[1]) && (inv_J2deg[1] < J2LimPos)) && ((J4LimNeg < inv_J4deg[1]) && (J4LimPos > inv_J4deg[1])))
  {
    rightHandedPossible = 1;
  } else
  {
    println("right handed solution is over angle limits");
  }
}
int checkLimitsForward(float J1angle, float J2angle, float J3distance, float J4angle) // has to be checked
{
  int forwardKinematicError = 1;
  // check if the solution is possible
  if ((J1LimNeg <= J1angle) && (J1angle <= J1LimPos) && (J2LimNeg <= J2angle) && (J2angle <= J2LimPos) && (J4LimNeg <= J4angle) && (J4LimPos >= J4angle) && (J3LimLower >= J3distance) && (J3distance >= J3LimUpper)) // LimLower = positive // LimUpper = negative
  {
    forwardKinematicError = 0;
  } else
  {
    forwardKinematicError = 0;
  }
  return forwardKinematicError;
}


public void inverseCalculate() {
  float x = float(cp5.get(Textfield.class, "xInverse").getText());
  float y = float(cp5.get(Textfield.class, "yInverse").getText());
  float z = float(cp5.get(Textfield.class, "zInverse").getText());
  float fi = float(cp5.get(Textfield.class, "fiInverse").getText());
  float accPerc = float(cp5.get(Textfield.class, "valueAccPercInverse").getText());
  float dccPerc = float(cp5.get(Textfield.class, "valueDccPercInverse").getText());
  float norSpeed = float(cp5.get(Textfield.class, "valueNorSpeedInverse").getText());
  float totTime = float(cp5.get(Textfield.class, "valueTotTimeInverse").getText());
  String controlMode = "";
  String solutionType = "";
  int debug = 0;
  int Vals[] = int(solutionButtons.getArrayValue());

  if (Vals[0] == 1) {
    solutionType = "l";
  }
  if (Vals[1] == 1) {
    solutionType = "r";
  }
  if (Vals[2] == 1) {
    solutionType = "b";
  }

  if (int(ddlCotrolModeInverse.getValue()) == 0) {
    controlMode = "a";
  }
  if (int(ddlCotrolModeInverse.getValue()) == 1) {
    controlMode = "s";
  }
  if (int(ddlCotrolModeInverse.getValue()) == 2) {
    controlMode = "t";
  }


  if (x !=x) {
    x = 0;
  }
  if (y !=y ) {
    y  = 0;
  }
  if (z !=z) {
    z = 0;
  }
  if (fi !=fi) {
    fi= 0;
  }
  if (accPerc !=accPerc) {
    accPerc = 0;
  }
  if (dccPerc !=dccPerc) {
    dccPerc = 0;
  }
  if (norSpeed !=norSpeed) {
    norSpeed = 0;
  }
  if (totTime !=totTime) {
    totTime = 0;
  }

  String message = "MJ X"+x+" Y"+y+" Z"+z+" F"+fi+" H" + solutionType + " C" + controlMode +" S" + norSpeed +" A"+ accPerc +" D" + dccPerc + " T" + totTime + " B" + debug ;
  println(message);

  InverseKinematicsSolver(x, y, z, fi);
  if (kinError == 1)
  {
    println("Inverse kinematics error - point is out of robot workspace");
    return;
  }
  println("Possible solutions: ");
  println("J1left: " + inv_J1deg[0] + " J2left: " + inv_J2deg[0] + " J3distance: " + inv_J3distance + " J4left: " + inv_J4deg[0]);
  println("J1right: " + inv_J1deg[1] + " J2right: " + inv_J2deg[1] + " J3distance: " + inv_J3distance + " J4right: " + inv_J4deg[1]);
  checkLimitsInverse();

  println("Step to do for solutions: ");
  int[] J1stepsToDo = {int(inv_J1deg[0] * J1stepsPerDeg) - J1stepsMaster, int(inv_J1deg[1] * J1stepsPerDeg) - J1stepsMaster};
  int[] J2stepsToDo = {int(inv_J2deg[0] * J2stepsPerDeg) - J2stepsMaster, int(inv_J2deg[1] * J2stepsPerDeg) - J2stepsMaster};
  int[] J3stepsToDo = {int(inv_J3distance * J3stepsPerMM) - J3stepsMaster, int(inv_J3distance * J3stepsPerMM) - J3stepsMaster};
  int[] J4stepsToDo = {int(inv_J4deg[0] * J4stepsPerDeg) - J4stepsMaster, int(inv_J4deg[1] * J4stepsPerDeg) - J4stepsMaster};

  if (leftHandedPossible == 1) {
    println("J1StDLeft: " + J1stepsToDo[0] + " J2StDLeft: " + J2stepsToDo[0] + " J3StDLeft: " + J3stepsToDo[0] + " J4StDLeft: " + J4stepsToDo[0]);
  }
  if (rightHandedPossible == 1) {
    println("J1StDRight: " + J1stepsToDo[1] + " J2StDRight: " + J2stepsToDo[1] + " J3StDRight: " + J3stepsToDo[1] + " J4StDRight: " + J4stepsToDo[1]);
  }



  int AxisMostStepsLeft = abs(J1stepsToDo[0]);
  if (abs(J2stepsToDo[0]) > AxisMostStepsLeft)
  {
    AxisMostStepsLeft = abs(J2stepsToDo[0]);
  }
  if (abs(J3stepsToDo[0]) > AxisMostStepsLeft)
  {
    AxisMostStepsLeft = abs(J3stepsToDo[0]);
  }
  if (abs(J4stepsToDo[0]) > AxisMostStepsLeft)
  {
    AxisMostStepsLeft = abs(J4stepsToDo[0]);
  }
  int AxisMostStepsRight = abs(J1stepsToDo[1]);
  if (abs(J2stepsToDo[1]) > AxisMostStepsRight)
  {
    AxisMostStepsRight = abs(J2stepsToDo[1]);
  }
  if (abs(J3stepsToDo[1]) > AxisMostStepsRight)
  {
    AxisMostStepsRight = abs(J3stepsToDo[1]);
  }
  if (abs(J4stepsToDo[1]) > AxisMostStepsRight)
  {
    AxisMostStepsRight = abs(J4stepsToDo[1]);
  }

  println("AxisMostStepsLeft: " + AxisMostStepsLeft);
  println("AxisMostStepsRight: " + AxisMostStepsRight);
  println("____________________");
}

public void inverseSend() {
  float x = float(cp5.get(Textfield.class, "xInverse").getText());
  float y = float(cp5.get(Textfield.class, "yInverse").getText());
  float z = float(cp5.get(Textfield.class, "zInverse").getText());
  float fi = float(cp5.get(Textfield.class, "fiInverse").getText());
  float accPerc = float(cp5.get(Textfield.class, "valueAccPercInverse").getText());
  float dccPerc = float(cp5.get(Textfield.class, "valueDccPercInverse").getText());
  float norSpeed = float(cp5.get(Textfield.class, "valueNorSpeedInverse").getText());
  float totTime = float(cp5.get(Textfield.class, "valueTotTimeInverse").getText());
  String controlMode = "";
  String solutionType = "";
  int debug = 0;
  int Vals[] = int(solutionButtons.getArrayValue());

  if (Vals[0] == 1) {
    solutionType = "l";
  }
  if (Vals[1] == 1) {
    solutionType = "r";
  }
  if (Vals[2] == 1) {
    solutionType = "b";
  }

  if (int(ddlCotrolModeInverse.getValue()) == 0) {
    controlMode = "a";
  }
  if (int(ddlCotrolModeInverse.getValue()) == 1) {
    controlMode = "s";
  }
  if (int(ddlCotrolModeInverse.getValue()) == 2) {
    controlMode = "t";
  }


  if (x !=x) {
    x = 0;
  }
  if (y !=y ) {
    y  = 0;
  }
  if (z !=z) {
    z = 0;
  }
  if (fi !=fi) {
    fi= 0;
  }
  if (accPerc !=accPerc) {
    accPerc = 0;
  }
  if (dccPerc !=dccPerc) {
    dccPerc = 0;
  }
  if (norSpeed !=norSpeed) {
    norSpeed = 0;
  }
  if (totTime !=totTime) {
    totTime = 0;
  }

  String message = "MJ X"+x+" Y"+y+" Z"+z+" F"+fi+" H" + solutionType + " C" + controlMode +" S" + norSpeed +" A"+ accPerc +" D" + dccPerc + " T" + totTime + " B" + debug ;
  println(message);

  InverseKinematicsSolver(x, y, z, fi);
  if (kinError == 1)
  {
    println("Inverse kinematics error - point is out of robot workspace");
    return;
  }
  println("Possible solutions: ");
  println("J1left: " + inv_J1deg[0] + " J2left: " + inv_J2deg[0] + " J3distance: " + inv_J3distance + " J4left: " + inv_J4deg[0]);
  println("J1right: " + inv_J1deg[1] + " J2right: " + inv_J2deg[1] + " J3distance: " + inv_J3distance + " J4right: " + inv_J4deg[1]);
  checkLimitsInverse();

  println("Step to do for solutions: ");
  int[] J1stepsToDo = {int(inv_J1deg[0] * J1stepsPerDeg) - J1stepsMaster, int(inv_J1deg[1] * J1stepsPerDeg) - J1stepsMaster};
  int[] J2stepsToDo = {int(inv_J2deg[0] * J2stepsPerDeg) - J2stepsMaster, int(inv_J2deg[1] * J2stepsPerDeg) - J2stepsMaster};
  int[] J3stepsToDo = {int(inv_J3distance * J3stepsPerMM) - J3stepsMaster, int(inv_J3distance * J3stepsPerMM) - J3stepsMaster};
  int[] J4stepsToDo = {int(inv_J4deg[0] * J4stepsPerDeg) - J4stepsMaster, int(inv_J4deg[1] * J4stepsPerDeg) - J4stepsMaster};

  if (leftHandedPossible == 1) {
    println("J1StDLeft: " + J1stepsToDo[0] + " J2StDLeft: " + J2stepsToDo[0] + " J3StDLeft: " + J3stepsToDo[0] + " J4StDLeft: " + J4stepsToDo[0]);
  }
  if (rightHandedPossible == 1) {
    println("J1StDRight: " + J1stepsToDo[1] + " J2StDRight: " + J2stepsToDo[1] + " J3StDRight: " + J3stepsToDo[1] + " J4StDRight: " + J4stepsToDo[1]);
  }



  int AxisMostStepsLeft = abs(J1stepsToDo[0]);
  if (abs(J2stepsToDo[0]) > AxisMostStepsLeft)
  {
    AxisMostStepsLeft = abs(J2stepsToDo[0]);
  }
  if (abs(J3stepsToDo[0]) > AxisMostStepsLeft)
  {
    AxisMostStepsLeft = abs(J3stepsToDo[0]);
  }
  if (abs(J4stepsToDo[0]) > AxisMostStepsLeft)
  {
    AxisMostStepsLeft = abs(J4stepsToDo[0]);
  }
  int AxisMostStepsRight = abs(J1stepsToDo[1]);
  if (abs(J2stepsToDo[1]) > AxisMostStepsRight)
  {
    AxisMostStepsRight = abs(J2stepsToDo[1]);
  }
  if (abs(J3stepsToDo[1]) > AxisMostStepsRight)
  {
    AxisMostStepsRight = abs(J3stepsToDo[1]);
  }
  if (abs(J4stepsToDo[1]) > AxisMostStepsRight)
  {
    AxisMostStepsRight = abs(J4stepsToDo[1]);
  }

  println("AxisMostStepsLeft: " + AxisMostStepsLeft);
  println("AxisMostStepsRight: " + AxisMostStepsRight);

  if (scaraConnected == 1) {
    scaraSerial.write(message+"\n");
  } else {
    println("SCARA controller disconnected");
  }
  println("____________________");
}

public void openFolder() {
  folderPath="";
  selectFolder("Select a folder to process:", "folderSelected");
}
public void OpenExistingFile() {
  selectInput("Select a file to process:", "fileSelected");
}

void folderSelected(File selection) {
  String path = "";
  if (selection == null) {
    println("No folder selected.");
  } else {
    path= selection.getAbsolutePath();
    println("selected folder: " + path);
    folderPath=path.trim();
  }
}
void fileSelected(File selection) {
  String path = "";
  if (selection == null) {
    println("No file selected.");
  } else {
    path = selection.getAbsolutePath();
    println("selected file: " + path);
    filePath = path;
    programArea.clear();
    readProgramToString();
  }
}

public void createAndOpenFile() {
  String fileName = cp5.get(Textfield.class, "fileName").getText();
  if (!folderPath.equals("") && !fileName.equals("")) {
    filePath = folderPath +  "\\"+  fileName + ".txt";
    println("opening: " + filePath);
  } else {
    println("No folder or file name");
    return;
  }
  programArea.clear();
  openedFile =  createWriter(filePath);
  openedFile.println("// PROGRAM START //");
  openedFile.flush();
  openedFile.close();
  readProgramToString();
}

public void closeFile() {
  if (openedFile != null) {
    openedFile.flush();
    openedFile.close();
    openedFile = null;
  }
  filePath = "";
  programLines=null;
  programArea.clear();
}

public void readProgramToString() {
  if (filePath.equals("")) {
    println("No file selected");
    return;
  }
  try {
    programLines = loadStrings(filePath);
  }
  catch (NullPointerException e) {
    e.printStackTrace();
  }
  updateProgramTextArea();
}


public void updateProgramTextArea() {
  programArea.clear();
  if (programLines == null) {
    return;
  }
  for (int i = 0; i < programLines.length; i++) {
    programArea.append(programLines[i]+"\n");
  }
}

public void saveProgramToFile() {
  if (filePath.equals("") || programLines == null) {
    println("No file selected");
    return;
  }
  saveStrings(filePath, programLines);
  println("Program saved to: " + filePath);
}

public void runProgram() {
  if (scaraConnected == 1 && nanoConnected == 1) {
    if (filePath.equals("") || programLines == null) {
      println("No file selected");
      programRunning = false;
      return;
    } else
    {
      firstLine = true;
      programRunning = true;
      previousModelMode= modelMode;
      modelMode = 3;
    }
  } else
    println("SCARA or NANO not connected");
}

public boolean sendLine() {

  int i = lineNumber;
  String message =programLines[i];
  String instruction = message.substring(0, 2);
  instruction = instruction.trim();
  println(message);
  if (instruction.equals("//")) {
    //println("comment line");
    ACKreceived = true;
    return true;
  }

  if (instruction.equals("CS") || instruction.equals("RL")) {
    if (nanoConnected == 1) {
      nanoSerial.write(message+"\n");
      return true;
    } else {
      println("fatal error - NANO controller disconnected");
      return false;
    }
  } else
  {
    if (scaraConnected == 1) {
      scaraSerial.write(message+"\n");
      return true;
    } else {
      println("fatal error - SCARA controller disconnected");
      return false;
    }
  }
}

public void pauseProgram() {
  if (programRunning) {
    programRunning = false;
    firstLine = false;
    ACKreceived = false;
  } else
  {
    println("No program running");
  }
}

public void stopProgram() {
  programRunning = false;
  firstLine = false;
  lineNumber = 0;
  ACKreceived = false;
  modelMode = previousModelMode;
}

public void enaTeach() {
  if (pathError()) {
    return;
  }

  int Vals[] = int(enableCheckbox.getArrayValue());
  String message = "EN A"+Vals[0]+" B"+Vals[1]+" C"+Vals[2]+" D"+Vals[3];
  enableCheckbox.deactivateAll();
  programLines = append(programLines, message);
  updateProgramTextArea();
}

public void homeTeach() {
  if (pathError()) {
    return;
  }
  int Vals[] = int(homeCheckbox.getArrayValue());
  String speed = cp5.get(Textfield.class, "homeSpeed").getText();
  String message = "HS A"+Vals[0]+" B"+Vals[1]+" C"+Vals[2]+" D"+Vals[3]+" S"+speed;
  homeCheckbox.deactivateAll();
  programLines = append(programLines, message);
  updateProgramTextArea();
}
public void robotPosTeach() {
  if (pathError()) {
    return;
  }
  String message = "RP ";
  programLines = append(programLines, message);
  updateProgramTextArea();
}
public void delayTeach() {
  if (pathError()) {
    return;
  }
  String waitTime = cp5.get(Textfield.class, "delayTime").getText();
  String message = "DL " + waitTime ;
  programLines = append(programLines, message);
  updateProgramTextArea();
}

public void pneuTeach() {
  if (pathError()) {
    return;
  }
  String nanoMessage1 = "";
  String nanoMessage2 = "";

  if (toggleAirSource) {
    nanoMessage1 = "P6Q0 S9T1 ";
  } else {
    nanoMessage1 = "P6Q1 S9T0 ";
  }

  if (toggleAirGripper) {
    nanoMessage2 = "U7V0";
  } else {
    nanoMessage2 = "U7V1";
  }
  String message = "RL " + nanoMessage1 + nanoMessage2;
  programLines = append(programLines, message);

  updateProgramTextArea();
}


public void forwardTeach() {
  if (pathError()) {
    return;
  }
  float J1angle = float(cp5.get(Textfield.class, "valueForwardJ1").getText());
  float J2angle = float(cp5.get(Textfield.class, "valueForwardJ2").getText());
  float J3distance = float(cp5.get(Textfield.class, "valueForwardJ3").getText());
  float J4angle = float(cp5.get(Textfield.class, "valueForwardJ4").getText());
  float accPerc = float(cp5.get(Textfield.class, "valueAccPerc").getText());
  float dccPerc = float(cp5.get(Textfield.class, "valueDccPerc").getText());
  float norSpeed = float(cp5.get(Textfield.class, "valueNorSpeed").getText());
  float totTime = float(cp5.get(Textfield.class, "valueTotTime").getText());
  String controlMode = "";
  int debug = 0;

  if (int(ddlCotrolMode.getValue()) == 0) {
    controlMode = "a";
  }
  if (int(ddlCotrolMode.getValue()) == 1) {
    controlMode = "s";
  }
  if (int(ddlCotrolMode.getValue()) == 2) {
    controlMode = "t";
  }


  if (J1angle !=J1angle) {
    J1angle = 0;
  }
  if (J2angle !=J2angle ) {
    J2angle  = 0;
  }
  if (J3distance !=J3distance) {
    J3distance = 0;
  }
  if (J4angle !=J4angle) {
    J4angle = 0;
  }
  if (accPerc !=accPerc) {
    accPerc = 0;
  }
  if (dccPerc !=dccPerc) {
    dccPerc = 0;
  }
  if (norSpeed !=norSpeed) {
    norSpeed = 0;
  }
  if (totTime !=totTime) {
    totTime = 0;
  }
  if (accPerc == 0 && controlMode == "s") {
    println("zero parameter error");
    println("____________________");
    return;
  }
  if (dccPerc == 0 && controlMode == "s" ) {
    println("zero parameter error");
    println("____________________");
    return;
  }
  if (norSpeed == 0 && controlMode == "s") {
    println("zero parameter error");
    println("____________________");
    return;
  }

  String message = "JJ A"+J1angle+" B"+J2angle+" C"+J3distance+" D"+J4angle+" E" + controlMode + " F" + norSpeed +" G" + accPerc +" H" + dccPerc + " I" + totTime + " J" + debug ;
  //println(message);
  ForwardKinematicsSolver(J1angle, J2angle, J3distance, J4angle);
  //println("x: " + fw_x + " y: " + fw_y + " z: " + fw_z + " fi: " + fw_fi);
  if ( checkLimitsForward(J1angle, J2angle, J3distance, J4angle)== 1 ) {
    println("Jog is out of robot axis limits");
    return;
  }

  programLines = append(programLines, message);
  updateProgramTextArea();
}

public void inverseTeach() {
  if (pathError()) {
    return;
  }
  float x = float(cp5.get(Textfield.class, "xInverse").getText());
  float y = float(cp5.get(Textfield.class, "yInverse").getText());
  float z = float(cp5.get(Textfield.class, "zInverse").getText());
  float fi = float(cp5.get(Textfield.class, "fiInverse").getText());
  float accPerc = float(cp5.get(Textfield.class, "valueAccPercInverse").getText());
  float dccPerc = float(cp5.get(Textfield.class, "valueDccPercInverse").getText());
  float norSpeed = float(cp5.get(Textfield.class, "valueNorSpeedInverse").getText());
  float totTime = float(cp5.get(Textfield.class, "valueTotTimeInverse").getText());
  String controlMode = "";
  String solutionType = "";
  int debug = 0;
  int Vals[] = int(solutionButtons.getArrayValue());

  if (Vals[0] == 1) {
    solutionType = "l";
  }
  if (Vals[1] == 1) {
    solutionType = "r";
  }
  if (Vals[2] == 1) {
    solutionType = "b";
  }

  if (int(ddlCotrolModeInverse.getValue()) == 0) {
    controlMode = "a";
  }
  if (int(ddlCotrolModeInverse.getValue()) == 1) {
    controlMode = "s";
  }
  if (int(ddlCotrolModeInverse.getValue()) == 2) {
    controlMode = "t";
  }


  if (x !=x) {
    x = 0;
  }
  if (y !=y ) {
    y  = 0;
  }
  if (z !=z) {
    z = 0;
  }
  if (fi !=fi) {
    fi= 0;
  }
  if (accPerc !=accPerc) {
    accPerc = 0;
  }
  if (dccPerc !=dccPerc) {
    dccPerc = 0;
  }
  if (norSpeed !=norSpeed) {
    norSpeed = 0;
  }
  if (totTime !=totTime) {
    totTime = 0;
  }

  String message = "MJ X"+x+" Y"+y+" Z"+z+" F"+fi+" H" + solutionType + " C" + controlMode +" S" + norSpeed +" A"+ accPerc +" D" + dccPerc + " T" + totTime + " B" + debug ;
  InverseKinematicsSolver(x, y, z, fi);
  if (kinError == 1)
  {
    println("Inverse kinematics error - point is out of robot workspace");
    return;
  }
  //println("Possible solutions: ");
  //println("J1left: " + inv_J1deg[0] + " J2left: " + inv_J2deg[0] + " J3distance: " + inv_J3distance + " J4left: " + inv_J4deg[0]);
  //println("J1right: " + inv_J1deg[1] + " J2right: " + inv_J2deg[1] + " J3distance: " + inv_J3distance + " J4right: " + inv_J4deg[1]);
  checkLimitsInverse();

  //println("Step to do for solutions: ");
  //int[] J1stepsToDo = {int(inv_J1deg[0] * J1stepsPerDeg) - J1stepsMaster, int(inv_J1deg[1] * J1stepsPerDeg) - J1stepsMaster};
  //int[] J2stepsToDo = {int(inv_J2deg[0] * J2stepsPerDeg) - J2stepsMaster, int(inv_J2deg[1] * J2stepsPerDeg) - J2stepsMaster};
  //int[] J3stepsToDo = {int(inv_J3distance * J3stepsPerMM) - J3stepsMaster, int(inv_J3distance * J3stepsPerMM) - J3stepsMaster};
  //int[] J4stepsToDo = {int(inv_J4deg[0] * J4stepsPerDeg) - J4stepsMaster, int(inv_J4deg[1] * J4stepsPerDeg) - J4stepsMaster};

  //if (leftHandedPossible == 1) {
  //  println("J1StDLeft: " + J1stepsToDo[0] + " J2StDLeft: " + J2stepsToDo[0] + " J3StDLeft: " + J3stepsToDo[0] + " J4StDLeft: " + J4stepsToDo[0]);
  //}
  //if (rightHandedPossible == 1) {
  //  println("J1StDRight: " + J1stepsToDo[1] + " J2StDRight: " + J2stepsToDo[1] + " J3StDRight: " + J3stepsToDo[1] + " J4StDRight: " + J4stepsToDo[1]);
  //}



  //int AxisMostStepsLeft = abs(J1stepsToDo[0]);
  //if (abs(J2stepsToDo[0]) > AxisMostStepsLeft)
  //{
  //  AxisMostStepsLeft = abs(J2stepsToDo[0]);
  //}
  //if (abs(J3stepsToDo[0]) > AxisMostStepsLeft)
  //{
  //  AxisMostStepsLeft = abs(J3stepsToDo[0]);
  //}
  //if (abs(J4stepsToDo[0]) > AxisMostStepsLeft)
  //{
  //  AxisMostStepsLeft = abs(J4stepsToDo[0]);
  //}
  //int AxisMostStepsRight = abs(J1stepsToDo[1]);
  //if (abs(J2stepsToDo[1]) > AxisMostStepsRight)
  //{
  //  AxisMostStepsRight = abs(J2stepsToDo[1]);
  //}
  //if (abs(J3stepsToDo[1]) > AxisMostStepsRight)
  //{
  //  AxisMostStepsRight = abs(J3stepsToDo[1]);
  //}
  //if (abs(J4stepsToDo[1]) > AxisMostStepsRight)
  //{
  //  AxisMostStepsRight = abs(J4stepsToDo[1]);
  //}

  //println("AxisMostStepsLeft: " + AxisMostStepsLeft);
  //println("AxisMostStepsRight: " + AxisMostStepsRight);

  programLines = append(programLines, message);
  updateProgramTextArea();
}

public void terminalTeach() {
  if (pathError()) {
    return;
  }
  String message = cp5.get(Textfield.class, "terminal").getText();
  programLines = append(programLines, message);
  cp5.get(Textfield.class, "terminal").clear();
  updateProgramTextArea();
}

boolean pathError() {
  if (filePath.equals("") || programLines == null) {
    println("No file selected");
    return true;
  }
  return false;
}

void showCS() {

  if (showCoordinateSys) {
    showCoordinateSys=false;
  } else {
    showCoordinateSys=true;
  }
}

public void  forwardModel() {
  modelMode = 1;
}
public void  inverseModel() {
  modelMode = 2;
}
public void  liveModel() {
  modelMode = 3;
}

public void  inverseModelLeft() {
  inverseModel = false;
}
public void  inverseModelRight() {
  inverseModel = true;
}

public void forwardLink(){
  if(forwardLinked){
  cp5.getController("forwardLink").setColorBackground(color(0, 100));
  cp5.getController("forwardLink").setColorActive(color(20, 85));
  cp5.getController("forwardLink").setColorForeground(color(20, 70));
  forwardLinked = false;
  return;
  }
  if(!forwardLinked){
   cp5.getController("forwardLink").setColorBackground(color(50, 255, 50, 255));
   cp5.getController("forwardLink").setColorActive(color(10, 255, 10, 255));
   cp5.getController("forwardLink").setColorForeground(color(100, 255, 100, 255));
  forwardLinked = true;
  return;
  }  
}


public void fanSend() {
  String nanoMessage1 = "";


  if (toggleFan) {
    nanoMessage1 = "P8Q0 ";
  } else {
    nanoMessage1 = "P8Q1 ";
  }
  String message ="RL "+ nanoMessage1  ;

  println(message);

  if (nanoConnected == 1) {
    nanoSerial.write(message + "\n");
  } else {
    println("NANO controller disconnected");
  }
  println("____________________");
}

public void fanTeach() {
  if (pathError()) {
    return;
  }
  String nanoMessage1 = "";


  if (toggleFan) {
    nanoMessage1 = "P8Q0 ";
  } else {
    nanoMessage1 = "P8Q1 ";
  }

  
  String message = "RL " + nanoMessage1;
  programLines = append(programLines, message);

  updateProgramTextArea();
}
