/*
  Written By: Alan Morrison
  Last Edited: 2020/01/28
*/

#include <Arduino.h>

#include <Wire.h>
#include "Servo.h"
#include "Adafruit_TCS34725.h"

// Variables that can change
unsigned long timeNow = 0;                                // Timestamp that updates each loop() iteration
const byte timeInterval = 1;                              // Time to pass until the next servo rotation
const byte angleInterval = 10;
int angleCurrent;                                         // Angle in microseconds = finer rotation resolution
int angleTarget;                                          // Angle in microseconds = finer rotation resolution
uint16_t redSensor, greenSensor, blueSensor, clearSensor; // Raw readings
volatile boolean state = false;                           // Flag to toggle interrupt on and off
byte clearRange = 0;
bool canIdentify = true; // Flag to toggle the Identification mode
unsigned long timeThen;  // Time tracking to add a delay after an MM is read
byte clearAdjust = 0;

bool backgroundCheck;

float yellowCheckOne;
float yellowCheckTwo;
bool yellowCheckResult;

float greenCheck;
float convSensor;
bool greenCheckResult;

float orangeCheckOne;
float orangeCheckTwo;
bool orangeCheckResult;

float redCheck;
bool redCheckResult;

float purpleCheck;
bool purpleCheckResult;

int skittlesCount[] = {
  0,
  0,
  0,
  0,
  0,
};

int backgroundSAMPLES[1][4] = {
    {89, 66, 59, angleTarget}};

// Variables that remain constant
const byte interruptPin = 2; // Signal input pin from TCS34725 interrupt

const int yellowAngle = 1810; // Angle to rotate the Servo in Microseconds
const int purpleAngle = 1420;  // Angle to rotate the Servo in Microseconds
const int greenAngle = 1230;  // Angle to rotate the Servo in Microseconds
const int orangeAngle = 2010;    // Angle to rotate the Servo in Microseconds
const int redAngle = 1600; // Angle to rotate the Servo in Microseconds

const bool TESTING = false;   // Set to true when gathering color data

// Instances a Servo object from the library and sets the sensing duration
// (integration time) and sensitivity (gain); see library options. Longer
// integration time plus lower gain = higher accuracy
// Due to the 700MS integration time missing Skittles, I had to move it down to 154MS, I might need to go lower, but this is still in testing
// Moved to 101MS again
// Tested 50MS, didn't improve the accuracy, in fact, it made it worse
Adafruit_TCS34725 colorSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_101MS, TCS34725_GAIN_1X);

// Instances a Servo object from the library
Servo skittleServo;

// For the compiler: isr() = function called when an interrupt occurs
void isr()
{
  state = true;
}

void getRawData_noDelay(uint16_t *redSensor, uint16_t *greenSensor, uint16_t *blueSensor, uint16_t *clearSensor)
{
  // getRawData() would cause a delay of the duration of the selected
  // integration time, thus blocking loop() like a delay() in the code;
  // but using the ISR (= interrupt) method, there is no more delay,
  // because one receives an interrupt once the integration is done and
  // loop() is no longer waiting for the sensor (* creates a pointer to
  // the memory address of a variable)
  *clearSensor = colorSensor.read16(TCS34725_CDATAL);
  *redSensor = colorSensor.read16(TCS34725_RDATAL);
  *greenSensor = colorSensor.read16(TCS34725_GDATAL);
  *blueSensor = colorSensor.read16(TCS34725_BDATAL);
}

void readSensor()
{

  if (state)
  {
    // A call to this function reads the RGB values into the matching
    // variables (& points to the memory address of a variable). The clear
    // value is not needed, but must be read as per the library code
    getRawData_noDelay(&redSensor, &greenSensor, &blueSensor, &clearSensor);

    backgroundCheck = false;

    int colorTrainingDistance = sqrt(pow(redSensor - backgroundSAMPLES[0][0], 2) + pow(greenSensor - backgroundSAMPLES[0][1], 2) + pow(blueSensor - backgroundSAMPLES[0][2], 2));

    if (colorTrainingDistance >= 2)
    {
      backgroundCheck = true;
    }

    // Dectivate the sensor's interrupt function after taking a reading
    colorSensor.clearInterrupt();
    state = false;
  }
}

int getColourDistance(int redSensor, int greenSensor, int blueSensor, int redSample, int greenSample, int blueSample)
{
  // Calculates the Euclidean distance between two RGB colors
  // https://en.wikipedia.org/wiki/Color_difference
  return sqrt(pow(redSensor - redSample, 2) + pow(greenSensor - greenSample, 2) + pow(blueSensor - blueSample, 2));
}

bool isGreen()
{
  greenCheckResult = false;
  //greenCheck = (((4.0f / 3.0f) * (float)redSensor) - (float)greenSensor); // Equation formerly used to determine Green
  convSensor = ((float)redSensor / (float)clearSensor) * 256.0f;

  if (convSensor < 110.2f) //greenCheck <= 51.5f) //
  {
    greenCheckResult = true;
  }

  return greenCheckResult;
}

bool isYellow()
{
  yellowCheckResult = false;
  convSensor = ((float)greenSensor / (float)clearSensor) * 256.0f;

  yellowCheckOne = ((277.0f / 342.0f) * (float)redSensor) - (float)greenSensor; // Equation used to determine Yellow
  yellowCheckTwo = ((float)greenSensor - (0.5f * (float)redSensor));
  /*
  if (yellowCheckOne <= 5.275)
  {
    yellowCheckResult = true;
  }
  else if (yellowCheckTwo >= 33)
  {
    yellowCheckResult = true;
  }
*/

  if (convSensor > 84.4f)
  {
    yellowCheckResult = true;
  }
  return yellowCheckResult;
}

bool isOrange()
{
  orangeCheckResult = false;
  orangeCheckOne = ((50.0f) / (27.0f) * (float)blueSensor) - ((1120.0f) / (27.0f));
  orangeCheckTwo = ((100.0f) / (67.0f) * (float)blueSensor) - ((1460.0f) / (67.0f));

  if (orangeCheckOne >= greenSensor && greenSensor >= orangeCheckTwo)
  {
    orangeCheckResult = true;
  }

  return orangeCheckResult;
}

bool isRed()
{
  redCheckResult = false;
  redCheck = ((3.1f * (float)blueSensor) - (float)redSensor);

  if (redCheck <= 94.4)
  {
    redCheckResult = true;
  }
  return redCheckResult;
}

bool isPurple()
{
  purpleCheckResult = false;
  purpleCheck = ((3.1f * (float)blueSensor) - (float)redSensor);

  if (purpleCheck > 94.4)
  {
    purpleCheckResult = true;
  }

  return purpleCheckResult;
}

void identifySample()
{
  if (backgroundCheck == true)
  {
    if (isGreen() == true)
    {
      angleTarget = greenAngle;
      skittlesCount[0] ++;
      if (TESTING != true)
      {
        Serial.println("Green!");
      }
    }

    else if (isYellow() == true)
    {
      angleTarget = yellowAngle;
      skittlesCount[1] ++;
      if (TESTING != true)
      {
        Serial.println("Yellow!");
      }
    }

    else if (isOrange() == true)
    {
      angleTarget = orangeAngle;
      skittlesCount[2] ++;
      if (TESTING != true)
      {
        Serial.println("Orange!");
      }
    }

    else if (isRed() == true)
    {
      angleTarget = redAngle;
      skittlesCount[3] ++;
      if (TESTING != true)
      {
        Serial.println("Red!");
      }
    }

    else if (isPurple() == true)
    {
      angleTarget = purpleAngle;
      skittlesCount[4] ++;
      if (TESTING != true)
      {
        Serial.println("Purple!");
      }
    }

    else 
    {
      angleTarget = purpleAngle;
      skittlesCount[4] ++;
      if (TESTING != true)
      {
        Serial.println("Error...");
      }
    }

    canIdentify = false;
    timeThen = millis() + 300;

    if (TESTING)
    {
      Serial.print(redSensor, DEC);
      Serial.print("\t");

      Serial.print(greenSensor, DEC);
      Serial.print("\t");

      Serial.print(blueSensor, DEC);
      Serial.print("\t");

      Serial.print(clearSensor, DEC);
      Serial.print("");
      Serial.println("");
    }
/*
    else if (TESTING != true)
    {
      Serial.print("Green: ");
      Serial.print(skittlesCount[0]);
      Serial.print("\tYellow: ");
      Serial.print(skittlesCount[1]);
      Serial.print("\tOrange: ");
      Serial.print(skittlesCount[2]);
      Serial.print("\tRed: ");
      Serial.print(skittlesCount[3]);
      Serial.print("\tPurple: ");
      Serial.print(skittlesCount[4]);
      Serial.println("Last Update (Millis): ");
      Serial.print(millis());
      Serial.println("");
    }
    */
  }
}

void rotateServo()
{
  // Check if it is time to rotate the servo another step towards the
  // target angle retrieved from the array. Depending on timeInterval,
  // the servo can be slowed down from its regular speed
  if (millis() - timeNow >= timeInterval)
  {
    // Create a new timestamp for the next step = loop() execution
    timeNow = millis();

    // Don't write to the servo after the target angle was reached
    if (angleCurrent == angleTarget)
    {
      canIdentify = true;
    }

    if (angleCurrent != angleTarget)
    {
      if (angleCurrent < angleTarget)
      {
        angleCurrent += angleInterval;
        skittleServo.writeMicroseconds(angleCurrent);
      }
      else if (angleCurrent > angleTarget)
      {
        angleCurrent -= angleInterval;
        skittleServo.writeMicroseconds(angleCurrent);
      }
    }
  }
}

void sensorInitialization()
{
  int red1;
  int green1;
  int blue1;
  int red2;
  int green2;
  int blue2;
  int red3;
  int green3;
  int blue3;
  int red4;
  int green4;
  int blue4;
  int red5;
  int green5;
  int blue5;
  readSensor();
  red1 = redSensor;
  green1 = greenSensor;
  blue1 = blueSensor;
  readSensor();
  red2 = redSensor;
  green2 = greenSensor;
  blue2 = blueSensor;
  readSensor();
  red3 = redSensor;
  green3 = greenSensor;
  blue3 = blueSensor;
  readSensor();
  red4 = redSensor;
  green4 = greenSensor;
  blue4 = blueSensor;
  readSensor();
  red5 = redSensor;
  green5 = greenSensor;
  blue5 = blueSensor;

  backgroundSAMPLES[0][0] = (red1 + red2 + red3 + red4 + red5) / 5;
  backgroundSAMPLES[0][1] = (green1 + green2 + green3 + green4 + green5) / 5;
  backgroundSAMPLES[0][2] = (blue1 + blue2 + blue3 + blue4 + blue5) / 5;
}

void setup()
{
  // TCS34725 interrupt output is active-LOW and open-drain
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), isr, FALLING);

  // Initialises the sensor; the LED is on, but could be switched via pin
  // A0 programmatically, or via connection to GND (LED off)
  colorSensor.begin();

  // Set persistence filter = generates an interrupt for every cycle,
  // regardless of the integration time limits
  colorSensor.write8(TCS34725_PERS, TCS34725_PERS_NONE);

  // Activate the sensor's interrupt function
  colorSensor.setInterrupt(true);

  // Uncomment for "color training" and checking purposes
  Serial.begin(9600);

  // Initialises the servo; connect its signal wire to pin 3; sets the pulse width for the SG90 Servo we are using
  skittleServo.attach(3, 500, 2400);

  // Rotates the servo to initial position (angle 0° in microseconds)
  skittleServo.writeMicroseconds(1500);

  // Seed both with an initial value to start with something. All samples
  // are related to an angle expressed in microseconds, stored in the
  // array. At the start, the current angle must be lower than the target
  // angle = the servo starts from the 0° position
  angleCurrent = 600;
  angleTarget = redAngle;

  sensorInitialization();
}

void loop()
{
  // A call to this function reads from the TCS34725 sensor. Unlike reading
  // from the sensor in the basic way, listening to the sensor's external
  // interrupt, the long integration time of 700ms - necessary to obtain
  // more accurate color readings - does no longer block loop() from
  // executing other code
  readSensor();

  if (canIdentify == true && timeNow >= timeThen)
  {
    // A call to this function iterates through the array to retrieve a
    // matching color sample
    identifySample();
  }

  // A call to this function rotates the servo based on the angle in
  // microseconds, also retrieved from the array
  rotateServo();
}