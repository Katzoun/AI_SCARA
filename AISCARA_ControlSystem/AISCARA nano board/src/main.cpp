#include <Arduino.h>

String receivedData = "";
String instruction;

String processedData;

char received;
unsigned long previousMillis = 0;
unsigned long previousMillis2 = 0;
const long interval = 1000; // interval at which to blink (milliseconds)
const long interval2 = 50;  // interval at which to read current sensors (milliseconds)
#define sensorPin1 A0
#define sensorPin2 A1
#define sensorPin3 A2
#define sensorPin4 A3

int sensor1Array[60];
int sensor2Array[60];
int sensor3Array[60];
int sensor4Array[60];

const int currentConstant = 185; // 185 for 5A sensor (185 mv/A)
const int numReadings = 10;
unsigned int readingNumber = 0;
int curReading = 0;

float sensor1Val = 0.0;
float sensor2Val = 0.0;
float sensor3Val = 0.0;
float sensor4Val = 0.0;
;

enum NanoStates
{
  WAITING_FOR_INSTRUCTION,
  READING_SERIAL_DATA,
  EXTRACTING_INSTRUCTION,
  PROCESSING_INSTRUCTION,
  CLEARING_BUFFER,

} NanoState;

enum Instructions
{
  NONE,
  RL,
  NC,
  CS
} Instruction;


void readSensors()
{

  if (curReading < numReadings)
  {
    sensor1Val = sensor1Val + analogRead(sensorPin1);
    sensor2Val = sensor2Val + analogRead(sensorPin2);
    sensor3Val = sensor3Val + analogRead(sensorPin3);
    sensor4Val = sensor4Val + analogRead(sensorPin4);
    curReading++;
  }
  else
  {
    sensor1Val = (((((sensor1Val / numReadings) * 5000.0) / 1023.0) - 2500) / currentConstant)*1000;
    sensor2Val = (((((sensor2Val / numReadings) * 5000.0) / 1023.0) - 2500) / currentConstant)*1000;
    sensor3Val = (((((sensor3Val / numReadings) * 5000.0) / 1023.0) - 2500) / currentConstant)*1000;
    sensor4Val = (((((sensor4Val / numReadings) * 5000.0) / 1023.0) - 2500) / currentConstant)*1000;

    sensor1Array[readingNumber]=sensor1Val;
    sensor2Array[readingNumber]=sensor2Val;
    sensor3Array[readingNumber]=sensor3Val;
    sensor4Array[readingNumber]=sensor4Val;
    readingNumber++;
    readingNumber = readingNumber % 60;

    sensor1Val = 0.0;
    sensor2Val = 0.0;
    sensor3Val = 0.0;
    sensor4Val = 0.0;
    curReading = 0;
  }
}

void processSerial()
{
  if (Serial.available() > 0 and processedData == "")
  {
    received = Serial.read();
    receivedData.concat(received);

    // Serial.println(received);
    if (received == '\n')
    {
      receivedData.trim();
      processedData = receivedData;
      receivedData = "";
      NanoState = EXTRACTING_INSTRUCTION; // leave this state
      // Serial.println("end of line detected");
      // Serial.println(processedData);
    }
  }
}

void setup()
{
  Serial.begin(9600);

  for (int i = 5; i < 14; i++)
  {
    pinMode(i, OUTPUT);
  }
  for (int i = 5; i < 14; i++)
  {
    digitalWrite(i, HIGH);
  }
  pinMode(sensorPin1, INPUT);
  pinMode(sensorPin2, INPUT);
  pinMode(sensorPin3, INPUT);
  pinMode(sensorPin4, INPUT);

  NanoState = WAITING_FOR_INSTRUCTION;
  Instruction = NONE;
}

void loop()
{
  unsigned long currentMillis = millis();

  // millis timer to read current sensors every x ms
  if (currentMillis - previousMillis2 >= interval2)
  {
    // fistPoint = micros();
    previousMillis2 = currentMillis;
    readSensors();
    // secondPoint = micros();
    // Serial.println(secondPoint - fistPoint);
  }

  switch (NanoState)
  {
  case WAITING_FOR_INSTRUCTION:
  {
    if (Serial.available() > 0)
    {
      NanoState = READING_SERIAL_DATA;
    }
  }
  break;

  case READING_SERIAL_DATA:
  {
    processSerial();
  }
  break;

  case EXTRACTING_INSTRUCTION:
  {
    instruction = processedData.substring(0, 2);
    if (instruction == "RL")
    {
      Instruction = RL;
      NanoState = PROCESSING_INSTRUCTION;
      break;
    }
    if (instruction == "NC")
    {
      Instruction = NC;
      NanoState = PROCESSING_INSTRUCTION;
      break;
    }
    if (instruction == "CS")
    {
      Instruction = CS;
      NanoState = PROCESSING_INSTRUCTION;
      break;
    }
    else
    {
      processedData = "";
      NanoState = WAITING_FOR_INSTRUCTION;
      break;
    }
  }
  break;

  case PROCESSING_INSTRUCTION:
  {
    switch (Instruction)
    {
    case RL:
    {
      String sendBack = "";
      digitalWrite(13, HIGH);
      int pinNumber1 = 0;
      int pinNumber2 = 0;
      int pinNumber3 = 0;
      int pinNumber4 = 0;


      int indexP = processedData.indexOf("P");
      int indexQ = processedData.indexOf("Q");
      int indexS = processedData.indexOf("S");
      int indexT = processedData.indexOf("T");

       int indexU = processedData.indexOf("U");
      int indexV = processedData.indexOf("V");
      int indexW = processedData.indexOf("W");
      int indexX = processedData.indexOf("X");


      pinNumber1 = processedData.substring(indexP + 1, indexQ).toInt();
      int pinState1 = processedData.substring(indexQ + 1, indexS).toInt();
      pinNumber2 = processedData.substring(indexS + 1, indexT).toInt();
      int pinState2= processedData.substring(indexT + 1, indexU).toInt();

      pinNumber3 = processedData.substring(indexU + 1, indexV).toInt();
      int pinState3 = processedData.substring(indexV + 1, indexW).toInt();
      pinNumber4 = processedData.substring(indexW + 1, indexX).toInt();
      int pinState4 = processedData.substring(indexX + 1).toInt();

      sendBack = "ACKRL ";

      if(pinNumber1 != 0)
      {
        digitalWrite(pinNumber1, pinState1);
        sendBack += "P" + String(pinNumber1) + "Q" + String(pinState1);
      }
      if(pinNumber2 != 0)
      {
        digitalWrite(pinNumber2, pinState2);
        sendBack += " S" + String(pinNumber2) + "T" + String(pinState2);
      }
       if(pinNumber3 != 0)
      {
        digitalWrite(pinNumber3, pinState3);
        sendBack += " U" + String(pinNumber3) + "V" + String(pinState3);
      }
      if(pinNumber4 != 0)
      {
        digitalWrite(pinNumber4, pinState4);
        sendBack += " W" + String(pinNumber4) + "X" + String(pinState4);
      }
      
      Serial.println(sendBack);
      NanoState = CLEARING_BUFFER;
    }
    break;

    case NC:
    {
      // check if nano is connected
      Serial.println("ACKNC S1");
      NanoState = CLEARING_BUFFER;
    }
    break;

    case CS:
    {
      // current sensor array contains 100 values for each sensor. 0 is the oldest, 99 is the newest. current sensor is read 2 times per sec. 500 ms per reading. 100 readings = 40 seconds of data
      int indexA = processedData.indexOf("A");
      int indexB = processedData.indexOf("B");
      int indexC = processedData.indexOf("C");
      int indexD = processedData.indexOf("D");
      int readA = processedData.substring(indexA + 1, indexA + 2).toInt();
      int readB = processedData.substring(indexB + 1, indexB + 2).toInt();
      int readC = processedData.substring(indexC + 1, indexC + 2).toInt();
      int readD = processedData.substring(indexD + 1, indexD + 2).toInt();
      Serial.println("ACKCS - PRINT READINGS STARTED FOR SELECTED DRIVERS");

      if (readA == 1)
      {
        Serial.println("DRIVER J1");
        for (int i = readingNumber; i < 60+readingNumber; i++)
        {
          Serial.print(sensor1Array[i%60]);
          Serial.print(",");
        }
        Serial.println();
      }
      if (readB == 1)
      {
        Serial.println("DRIVER J2");
        for (int i = readingNumber; i < 60+readingNumber; i++)
        {
          Serial.print(sensor2Array[i%60]);
          Serial.print(",");
        }
        Serial.println();
      }
      if (readC == 1)
      {
        Serial.println("DRIVER J3");
        for (int i = readingNumber; i < 60+readingNumber; i++)
        {
          Serial.print(sensor3Array[i%60]);
          Serial.print(",");
        }
        Serial.println();
      }
      if (readD == 1)
      {
        Serial.println("DRIVER J4");
        for (int i = readingNumber; i < 60+readingNumber; i++)
        {
          Serial.print(sensor4Array[i%60]);
          Serial.print(",");
        }
        Serial.println();
      }
      NanoState = CLEARING_BUFFER;
    }
    break;

    case NONE:
    {
      // Serial.println("NONE");
      NanoState = CLEARING_BUFFER;
    }
    break;

    } // end of instruction switch
  }
  break;

  case CLEARING_BUFFER:
  {
    processedData = "";
    NanoState = WAITING_FOR_INSTRUCTION;
  }
  break;

  }

  // blink led
  if (currentMillis - previousMillis >= interval) // blink led
  {
    previousMillis = currentMillis;
    digitalWrite(13, !digitalRead(13)); // led is sign of life
  }
}
