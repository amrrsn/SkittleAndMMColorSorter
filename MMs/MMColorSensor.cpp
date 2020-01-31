/*
  Written By: Alan Morrison
  Last Edited: 2020/01/29
*/

#include <Arduino.h>

#include <Wire.h>
#include "Servo.h"
#include "Adafruit_TCS34725.h"

unsigned long timeNow = 0;                                // Timestamp that updates each loop() iteration
const byte timeInterval = 1;                              // Time to pass until the next servo rotation
const byte angleInterval = 10;                            // MicroSeconds to increase the AngleCurrent for servo rotation
int angleCurrent;                                         // Angle in microseconds = finer rotation resolution
int angleTarget;                                          // Angle in microseconds = finer rotation resolution
uint16_t redSensor, greenSensor, blueSensor, clearSensor; // Raw readings
volatile boolean state = false;                           // Flag to toggle interrupt on and off
bool canIdentify = true;                                  // Flag to toggle the Identification mode
unsigned long timeThen;                                   // Time tracking to add a delay after an MM is read

bool backgroundCheck;      // Tracks whether or not the color sensor is reading the background or an MM

float yellowCheckOne;
float yellowCheckTwo;
bool yellowCheckResult;

float greenCheck;
float convSensor;
bool greenCheckResult;

float orangeCheck;
bool orangeCheckResult;

bool blueCheckResult;
float blueCheck;

bool brownCheckResult;
float brownCheckOne;
float brownCheckTwo;

bool redCheckResult;
float redCheck;

int mmCount[6] = {  // Contains the tracking data for counting the various MM Colors
  0,
  0,
  0,
  0,
  0,
  0,
};

int backgroundSAMPLES[1][3] = {   // Contains the data for later background calibration, values are unlikely to go above 255 with the current integration settings
    {89, 66, 59}};                // (given the fact that the background is black tape), but I kept it an int to future-proof

// Variables that remain constant
const byte interruptPin = 2; // Signal input pin from TCS34725 interrupt

const int orangeAngle = 1330; // Angle to rotate the Servo in Microseconds
const int greenAngle = 2070;  // Angle to rotate the Servo in Microseconds
const int redAngle = 1120;    // Angle to rotate the Servo in Microseconds
const int yellowAngle = 1880; // Angle to rotate the Servo in Microseconds
const int blueAngle = 1590;   // Angle to rotate the Servo in Microseconds
const int brownAngle = 850;  // Angle to rotate the Servo in Microseconds

const bool TESTING = false;   // Tag to enable/disable certain debug features.  Implemented after I spent 3 hours troubleshooting the debug code that caused the main code to fail.

// Instances a Servo object from the library and sets the sensing duration
// (integration time) and sensitivity (gain); see library options. Longer
// integration time plus lower gain = higher accuracy
// Due to the 700MS integration time missing MMs, I had to move it down to 154MS, I might need to go lower, but this is still in testing
// Moved to 101MS again
// Testing 50MS
Adafruit_TCS34725 colorSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_101MS, TCS34725_GAIN_1X);

// Servo Info: 0 degrees = 1500 microseconds; +90 degrees = 2000 microseconds; -90 degrees = 1000 microseconds
// Instances a Servo object from the library
Servo mmServo;

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

    // Sets the bachground check to false, just in case something happened and it didn't get reset earlier
    backgroundCheck = false;

    // Uses a somewhat modified version of getColorDistance() to check the distance from the background color
    // At one point this check was in identifySample(), but was moved during a debug session
    // Now there's no particular reason for it o be here specifically, but w/e it works
    int backgroundColorDistance = sqrt(pow(redSensor - backgroundSAMPLES[0][0], 2) + pow(greenSensor - backgroundSAMPLES[0][1], 2) + pow(blueSensor - backgroundSAMPLES[0][2], 2));

    // Unless the background color is variable, the color distance should almost always be 0, but I used a range of 2 to allow for some small variations
    if (backgroundColorDistance >= 2)
    {
      backgroundCheck = true;
    }

    // Dectivate the sensor's interrupt function after taking a reading
    colorSensor.clearInterrupt();
    state = false;
  }
}

int getColorDistance(int redSensor, int greenSensor, int blueSensor, int redSample, int greenSample, int blueSample)
{
  // Calculates the Euclidean distance between two RGB colors
  // https://en.wikipedia.org/wiki/Color_difference
  return sqrt(pow(redSensor - redSample, 2) + pow(greenSensor - greenSample, 2) + pow(blueSensor - blueSample, 2));
}

bool isBlue()
{
  blueCheckResult = false;
  blueCheck = ((float)redSensor - 0.2f * (float)blueSensor);

  if (blueCheck <= 72.1f)
  {
    blueCheckResult = true;
  }

  return blueCheckResult;
}

bool isGreen()
{
  greenCheckResult = false;
  greenCheck = ((float)redSensor - 1.1f * (float)blueSensor);

  if (greenCheck <= 28.0f)
  {
    greenCheckResult = true;
  }

  return greenCheckResult;
}

bool isYellow()
{
  yellowCheckResult = false;

  yellowCheckOne = (0.8f * (float)redSensor - (float)greenSensor);
  yellowCheckTwo = ((float)greenSensor - 0.3f * (float)redSensor);

  if (yellowCheckOne <= 9.75f)
  {
    yellowCheckResult = true;
  }

  else if (yellowCheckTwo >= 43.0f)
  {
    yellowCheckResult = true;
  }

  return yellowCheckResult;
}

bool isBrown()
{
  brownCheckResult = false;
  float convRedSensor = ((float)redSensor / (float)clearSensor) * 256.0f;     // Take the clear value into account, sometimes helps split the specific color candy on a 2d plot 
  float convGreenSensor = ((float)greenSensor / (float)clearSensor) * 256.0f; // Take the clear value into account, sometimes helps split the specific color candy on a 2d plot
  brownCheckOne = ((0.34f * convRedSensor) + convGreenSensor);
  brownCheckTwo = ((float)greenSensor - (0.23f * (float)redSensor));

  if (brownCheckOne >= 116)
  {
    brownCheckResult = true;
  }

  else if (brownCheckTwo >= 37.8)
  {
    brownCheckResult = true;
  }

  return brownCheckResult;
}


bool isOrange()
{
  orangeCheckResult = false;
  float convBlueSensor = ((float)blueSensor / (float)clearSensor) * 256.0f;   // Take the clear value into account, sometimes helps split the specific color candy on a 2d plot
  float convGreenSensor = ((float)greenSensor / (float)clearSensor) * 256.0f; // Take the clear value into account, sometimes helps split the specific color candy on a 2d plot
  orangeCheck = (convBlueSensor - (0.82f * convGreenSensor));

  if (orangeCheck < 5)
  {
    orangeCheckResult = true;
  }

  return orangeCheckResult;
}

bool isRed()
{
  redCheckResult = false;
  float convBlueSensor = ((float)blueSensor / (float)clearSensor) * 256.0f;   // Take the clear value into account, sometimes helps split the specific color candy on a 2d plot
  float convGreenSensor = ((float)greenSensor / (float)clearSensor) * 256.0f; // Take the clear value into account, sometimes helps split the specific color candy on a 2d plot
  redCheck = (convBlueSensor - (0.82f * convGreenSensor));                    // Equation found to seperate the Red MMs from the rest

  if (redCheck > 5)
  {
    redCheckResult = true;
  }

  return redCheckResult;
}


void identifySample()
{
  if (backgroundCheck == true)
  {
    if (isBlue() == true)
    {
      angleTarget = blueAngle;
      mmCount[0] ++;
      if (TESTING != true)
      {
        Serial.println("Blue!");
      }
    }

    else if (isGreen() == true)
    {
      angleTarget = greenAngle;
      mmCount[1] ++;
      if (TESTING != true)
      {
        Serial.println("Green!");
      }
    }

    else if (isYellow() == true)
    {
      angleTarget = yellowAngle;
      mmCount[2] ++;
      if (TESTING != true)
      {
        Serial.println("Yellow!");
      }
    }

    else if (isBrown() == true)
    {
      angleTarget = brownAngle;
      mmCount[3] ++;
      if (TESTING != true)
      {
        Serial.println("Brown!");
      }
    }

    else if (isOrange() == true)
    {
      angleTarget = orangeAngle;
      mmCount[4] ++;
      if (TESTING != true)
      {
        Serial.println("Orange!");
      }
    }

    else if (isRed() == true)
    {
      angleTarget = redAngle;
      mmCount[5] ++;
      if (TESTING != true)
      {
        Serial.println("Red!");
      }
    }

    else
    {
      angleTarget = brownAngle;
      mmCount[3] ++;
      if (TESTING != true)
      {
        Serial.println("Error...");
      }
    }

    canIdentify = false;
    timeThen = millis() + 300;

    // Prints data in a nice format for directly copying and pasting to excel for data analysis
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
      //Serial.println(millis());
    }

    // Gives a Serial Monitor Counter for each Color Type, can be annoying when running tests
    /*
    else if (TESTING != true)
    {
      Serial.print("Blue: ");
      Serial.print(mmCount[0]);
      Serial.print("\tGreen: ");
      Serial.print(mmCount[1]);
      Serial.print("\tYellow: ");
      Serial.print(mmCount[2]);
      Serial.print("\tBrown: ");
      Serial.print(mmCount[3]);
      Serial.print("\tOrange: ");
      Serial.print(mmCount[4]);
      Serial.print("\tRed: ");
      Serial.print(mmCount[5]);
      Serial.print("\tLast Update (Millis): ");
      Serial.println(millis());
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

    // If the target angle has been reached, allow the identification code to run once again
    if (angleCurrent == angleTarget)
    {
      canIdentify = true;
    }

    // Don't write to the servo after the target angle was reached
    if (angleCurrent != angleTarget)
    {
      if (angleCurrent < angleTarget)
      {
        angleCurrent += angleInterval;
        mmServo.writeMicroseconds(angleCurrent);
      }
      else if (angleCurrent > angleTarget)
      {
        angleCurrent -= angleInterval;
        mmServo.writeMicroseconds(angleCurrent);
      }
    }
  }
}

void sensorCalibration()
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
  mmServo.attach(3, 500, 2400);

  // Rotates the servo to initial position (angle 0° in microseconds)
  mmServo.writeMicroseconds(1500);

  // Seed both with an initial value to start with something. All samples
  // are related to an angle expressed in microseconds, stored in the
  // array. At the start, the current angle must be lower than the target
  // angle = the servo starts from the 0° position
  angleCurrent = 600;
  angleTarget = 1560;

  // Calibrates the sensor by taking 5 reads of the background color, averaging them, and then applying them to the proper areas
  sensorCalibration();
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