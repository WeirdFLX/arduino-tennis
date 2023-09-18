#include <ESC.h>

#include <Wire.h>
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <A4988.h>

#include <TM1637Display.h>
#include <BasicStepperDriver.h>
#include <EEPROM.h>
#define MOTOR_STEPS 200
#define RPM 120
// Microstepping mode. If you hardwired it to save pins, set to the same value here.
#define MICROSTEPS 1

#define DIR 23
#define STEP 22

// #define SLEEP 13 // optional (just delete SLEEP from everywhere if not used)

/*
 * Choose one of the sections below that match your board
 */

#include "DRV8834.h"
#define M0 10
#define M1 11
DRV8834 stepperX(MOTOR_STEPS, DIR, STEP, M0, M1);
ESC upperESC(40, 1000, 2000, 800);
ESC lowerESC(39, 1000, 2000, 800);

long timer;

// Unterklasse für den Status des Programmes
enum programmStatusEnum
{
  ballOneSelected,
  ballTwoSelected,
  ready,
  reset,
  debug,
  debugDone
};
// Status des Programmes
programmStatusEnum programmStatus;

// Unterklasse für Debugmodus um Werte zu ändern
enum debugParameterStatusEnum
{
  initVerHor,
  verticalMin,
  verticalMax,
  horizontalMin,
  horizontalMax,
  initAbs,
  absSpeedMin,
  absSpeedMax,
  absSpinMin,
  absSpinMax,
  done
};
// Status der Debugfunktion
debugParameterStatusEnum debugParameterStatus;

// Technische Trommelwerte
enum TrommelStatusEnum
{
  man,
  one,
  two,
  set
};
// Trommelstatus
TrommelStatusEnum trommelStatus;

// Variablen für das LCD-Display
LiquidCrystal_I2C lcd(0x27, 20, 4);
// Icon Linie Links
byte line_left[8] = {B10000, B10000, B10000, B10000, B10000, B10000, B10000, B10000};
// Icon Linie Rechts
byte line_right[8] = {B00001, B00001, B00001, B00001, B00001, B00001, B00001, B00001};
// Icon Ball
byte ball[8] = {B00000, B00000, B00100, B01110, B01110, B00100, B00000, B00000};
// Icon Leer
byte empty[8] = {B00000, B00000, B00000, B00000, B00000, B000000, B00000, B00000};
// Icon Arrow
byte arrow[8] = {B00000, B00110, B01100, B11111, B11111, B01100, B00110, B00000};
// Breite des Feldes
int lcdFieldWidth = 6;

// Steppersettings
int stepsprorev(200);
int revspeed = 300;
// Variablen für Stepper
// BasicStepperDriver stepperX(stepsprorev, 23, 22);
int xStepperold;
BasicStepperDriver stepperY(stepsprorev, 26, 27);
BasicStepperDriver stepperR(stepsprorev, 24, 25);
// Stepperlogging
unsigned long xStepper;
unsigned long xStepperLast;
unsigned long Rstepper;
unsigned long Rstepperlast;
unsigned long timeBallThrown;
bool ballThrown = false;
int Rspeed;

bool motorInitialisiert;

// Position des ausgewählten Balls
int ballPosition[2] = {1, 0};
bool handleBallPositionInterrupt;
bool ballPositionDT;

// Pin-Deklarationen
const byte encBallPositionCLK(2);
const byte encBallPositionDT(4);
const byte encBallPositionSW(5);
long encPositionTime;
bool encBallPosiotionCLKold;

// Variablen und Display für Ballgeschwindigkeit
TM1637Display ballSpeedDisplay = TM1637Display(53, 52);
int ballSpeed;
bool handleBallSpeedInterrupt;
bool ballSpeedDT;
// Pin-Deklarationen
const byte encBallSpeedCLK(3);
const byte encBallSpeedDT(45);
const byte encBallSpeedSW(6);
unsigned long encSpeedTime;
int oldBallSpeed;
byte encSpeedbool; // nUR JEDER zweite
// Variablen und Display für Ballspin
TM1637Display ballSpinDisplay = TM1637Display(51, 50);
int ballSpin;
bool handleBallSpinInterrupt;
bool ballSpinDT;
int encSpinbool;
int oldBallSpin;
long encSpinTime;

// Pin-Deklarationen
const byte encBallSpinCLK(18);
const byte encBallSpinDT(34);
// für Debug-Modus
const byte encBallSpinSW(29);

// Variablen und Display für Ballintervall
TM1637Display ballIntervalDisplay = TM1637Display(49, 48);
int ballInterval;
bool handleBallIntervallInterrupt;
bool ballIntervallDT;
// Pin-Deklarationen

const byte encBallIntervalDT(47);

// Pin-Deklaration für geshareten 19er
const byte encBallIntervalCLK(41);
const byte pinLichtschranke(44);
const byte encBallIntervalSW(36);
const byte pinStart(35);
bool sharedInterArrayOld[5] = {0, 0, 0, 0, 0};
bool sharedInterArray[5] = {0, 0, 0, 0, 0};

bool handleStartInterupt;
// Variablen für Lichtschranke
bool handleLichtschrankeInterrupt;
// Pin-Deklaration
// const byte lichtschranke(19);

/* Werte der gespeicherten Bälle
 * 0: X-Position
 * 1: Y-Position
 * 2: Geschwindigkeit
 * 3: Spin
 */
int ballOneValues[4], ballTwoValues[4] = {0, 0, 0, 0};

// Wurde der Recorder zur Einstellung der Ballposition gedrückt?
bool pressMethodUsed;

// Variablen für Debugmodus
int lastDebugPressState = LOW;
int currentDebugPressState;
unsigned long debugPressedTime = 0;
unsigned long debugReleasedTime = 0;
const int LONG_PRESS_TIME = 6000;

// Einstellungsvariablen, nur durch DEBUG änderbar

bool debugValSaved = false;
int debugCursorPos[2] = {0, 0};

/*int verticalMinVal;
int verticalMaxVal;
int horizontalMinVal;
int horizontalMaxVal;
int absSpeedMinVal;
int absSpeedMaxVal;
int absSpinMinVal;
int absSpinMaxVal;*/

int curDebugValues[8] = {0, 100, 0, 100, 0, 70, 0, 70};
int oldDebugValues[8] = {0, 0, 0, 0, 0, 0, 0, 0};
// Wurde ein Wert geändert?
bool debugValueChanged[8] = {false, false, false, false, false, false, false, false};

// Sollwerte für Trommeln & Position
// Ohne ges. Ball       Vert, Hor, Lower, Upper
float sollTrommelMan[4] = {0, 0, 0, 0};
// Ball1
float sollTrommelOne[4] = {0, 0, 0, 0};
// Ball2
float sollTrommelTwo[4] = {0, 0, 0, 0};

float absSpeed[3] = {0, 0, 0};

float absSpin[3] = {0, 0, 0};

// Setup-Methode
void setup()
{

  // Serielle Ausgabe verbinden für ggf. Debugging
  Serial.begin(9600);
  Wire.begin();
  xStepperLast = 0;
  // Schrittmotoren Parameter
  revspeed = 30;
  //  stepperX.begin(revspeed, 1);
  stepperY.begin(revspeed, 1);
  stepperR.begin(revspeed, 1);

  //  stepperX.move(2);

  // Pins setzen und Interrupts anbinden
  pinMode(encBallPositionDT, INPUT);
  pinMode(encBallPositionSW, INPUT);
  pinMode(encBallSpeedDT, INPUT);
  pinMode(encBallSpinDT, INPUT_PULLUP);
  pinMode(encBallSpinSW, INPUT);
  pinMode(encBallIntervalDT, INPUT);
  pinMode(pinLichtschranke, INPUT);
  // pinMode(lichtschranke, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encBallPositionCLK), encoderBallPositionInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encBallSpeedCLK), encoderBallSpeedInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(encBallSpinCLK), encoderBallSpinInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(19), sharedInterrupt, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(lichtschranke), lichtschrankeInterrupt, CHANGE);

  // attachInterrupt(digitalPinToInterrupt(encBallPositionSW),pressMethod,RISING);
  // Daten aus Eprom aulesen; aktuell temporär benutzt
  ballSpeed = 10;
  ballSpin = 0;
  ballInterval = 2;

  // Displays initialisiseren
  initializeLcdDisplay();
  resetLcdField();
  initializeSevenSegment();
  // Programmstatus setzen
  programmStatus = ballOneSelected;
  debugParameterStatus = initVerHor;
  trommelStatus = man;
  A4988(short(200), short(23), short(22));
  stepperX.setSpeedProfile(BasicStepperDriver::LINEAR_SPEED, 6000, 6000);
  // Serial.println(x);
  stepperX.setRPM(300);
  upperESC.arm();
  lowerESC.arm();
}

// Main-Loop
void loop()
{
  unsigned wait_time_micros = stepperX.nextAction();

  interruptHandler();
  // trommelBerechnen();
  if (programmStatus == debug)
  {
    debugInit();
    debugCheckSave();
  }
  else if (programmStatus == debugDone)
  {
    // Speichern der geänderten Werte, nur falls geändert

    trommelBerechnen();
    updateDebugParameterStatus();
    programmStatus = reset;
  }
  else
  {

    pressMethod();
    if (programmStatus == reset)
    {
      resetLcdField();
      programmStatus = ballOneSelected;
    }

    trommelAdjust();
  }
}

/* Methode wenn der Recorder zur Einstellung der Ballposition gedrückt wird TODO Rework
 */
void pressMethod()
{
  if (digitalRead(encBallPositionSW) == HIGH && pressMethodUsed)
  {
    pressMethodUsed = false;

    if (programmStatus == ballOneSelected)
    {
      ballOneValues[0] = ballPosition[0];
      ballOneValues[1] = ballPosition[1];
      ballOneValues[2] = ballSpeed;
      ballOneValues[3] = ballSpin;
      programmStatus = ballTwoSelected;
      updateBallPosition(true);
      // Serial.println("Ball One Saved");
    }
    else if (programmStatus == ballTwoSelected)
    {
      ballTwoValues[0] = ballPosition[0];
      ballTwoValues[1] = ballPosition[1];
      ballTwoValues[2] = ballSpeed;
      ballTwoValues[3] = ballSpin;
      programmStatus = ready;
      updateBallPosition(true);
      // Serial.println("Ball Two Saved");
    }
    else
    {
      // gespeicherte Werte zurücksetzen
      for (int i = 0; i < 4; i++)
      {
        ballOneValues[i] = 0;
        ballTwoValues[i] = 0;
      };
      programmStatus = reset;
      // Serial.println("Balls resetted");
    }
  }
  if (digitalRead(encBallPositionSW) == LOW)
  {
    pressMethodUsed = true;
  }
  trommelBerechnen();
}

//--- Abschnitt: Balleigenschaften aktualisieren
/* Methode um die Position des Balles zu aktualisieren
 * @param overrideDT ballPositionDT wird ignoriert wenn der Wert mit true gefüllt ist, es wird um 1 erhöht.
 */
void updateBallPosition(bool overrideDT)
{
  Serial.print(ballPositionDT);
  deleteBall();
  drawBallNumber();
  if (overrideDT)
  {
    ballPosition[0] = updatePropertyValue(ballPosition[0], 1, overrideDT);
  }
  else
  {
    if (encPositionTime + 10 < millis())
      ballPosition[0] = updatePropertyValue(ballPosition[0], 1, ballPositionDT);
    encPositionTime = millis();
  }
  wrapBallPosition();
  drawBall();
  trommelBerechnen();
}

/* Methode um die Position des Balles auf die Breite des Feldes anzupassen
 * Stichwort Zeilenumbruch, dadurch kann der Ball in x- und y-Achse bewegt werden.
 */
void wrapBallPosition()
{
  // x (ballPosition[0]) wrappen
  if (ballPosition[0] > lcdFieldWidth - 1)
  {
    ballPosition[1]++;
    ballPosition[0] = 1;
  }
  else if (ballPosition[0] < 1)
  {
    ballPosition[1]--;
    ballPosition[0] = lcdFieldWidth - 1;
  }
  // y (ballPosition[1]) wrappen
  if (ballPosition[1] > 3)
  {
    ballPosition[1] = 0;
  }
  else if (ballPosition[1] < 0)
  {
    ballPosition[1] = 3;
  }
}

/* Methode zur aktualisierung der Ball-Geschwindigkeit,
 * sowie des dazugehörigen Displays
 */
void updateBallSpeed()
{
  if (encSpeedbool < 2)
  {
    encSpeedbool = encSpeedbool + 1;
  }
  if ((encSpeedbool >= 2))
  {
    if (encSpeedTime + 2 < millis())
    {
      oldBallSpeed = ballSpeed;
      ballSpeed = updatePropertyValue(ballSpeed, 2, ballSpeedDT);
      encSpeedTime = millis();
      ballSpeed = overflowCorrection(ballSpeed, 100, 10);
      encSpeedbool = 0;
    }
  }
  if (oldBallSpeed != ballSpeed)
  {
    refreshSevenSegmentDisplay(ballSpeedDisplay, ballSpeed);
  }
  trommelBerechnen();
}

/* Methode zur aktualisierung des Ball-Spins,
 * sowie des dazugehörigen Displays
 */
void updateBallSpin()
{ // Serial.print(ballSpinDT);
  if (encSpinbool < 2)
  {
    encSpinbool = encSpinbool + 1;
  }
  if ((encSpinbool >= 2) || ballSpinDT)
  {
    if (encSpinTime + 10 < millis())
    {
      oldBallSpin = ballSpin;
      encSpinTime = millis();
      ballSpin = updatePropertyValue(ballSpin, 10, ballSpinDT);
      ballSpin = overflowCorrection(ballSpin, 100, -100);
      encSpinbool = 0;
    }
    if (oldBallSpin != ballSpin)
    {
      refreshSevenSegmentDisplay(ballSpinDisplay, ballSpin);
    }
    trommelBerechnen();
  }
}
/* Methode zur aktualisierung des Ball-Intervalls,
 * sowie des dazugehörigen Displays
 */
void updateBallInterval()
{
  int oldBallInterval = ballInterval;
  ballInterval = updatePropertyValue(ballInterval, 1, ballIntervallDT);
  ballInterval = overflowCorrection(ballInterval, 10, 2);
  if (oldBallInterval != ballInterval)
  {
    refreshSevenSegmentDisplay(ballIntervalDisplay, ballInterval);
  }
  trommelBerechnen();
}

/* Methode um eine Balleigenschaft zu
 * aktualisieren.
 *
 * @param value Wert der aktualisiert werden soll
 * @param step Wert um den erhöht / reduziert werden soll
 * @param increase Soll der Wert erhöht oder reduziert werden?
 * @return berechneter Wert
 */
int updatePropertyValue(int value, int step, boolean increase)
{
  if (increase)
  {
    return value + step;
  }
  else
  {
    return value - (step);
  }
}

/* Methode zur Prüfung und Korrektur des Wertes
 * auf Basis des Grenzbereiches
 *
 * @param value Wert der verglichen wird
 * @param upper Obere Grenze
 * @param lower Untere Grenze
 * @return Wert oder Grenzwert
 */
int overflowCorrection(int value, int upper, int lower)
{
  if (value > upper)
  {
    return upper;
  }
  else if (value < lower)
  {
    return lower;
  }
  else
  {
    return value;
  }
}

//--- Abschnitt: Displays
/* Methode zur Initialisierung eines Seven-Segment Displays
 * @param DSPclk Pin an den CLK des Displays angeschlossen ist
 * @param DSPdio Pin an den DIO des Displays angeschlossen ist
 * @param value anzuzeigender Wert
 * @return Das initialisierte Display
 */
TM1637Display createSevenSegmentDisplay(int DSPclk, int DSPdio, int value)
{
  TM1637Display display = TM1637Display(DSPclk, DSPdio);
  display.setBrightness(0x0f);
  display.setSegments(4);
  display.showNumberDec(value);
  Serial.println("Display erstellt");
  return display;
}

/* Methode zur Intialisierung der Seven-Segment Displays
 */
void initializeSevenSegment()
{
  ballSpeedDisplay = createSevenSegmentDisplay(53, 52, ballSpeed);
  ballSpinDisplay = createSevenSegmentDisplay(51, 50, ballSpin);
  ballIntervalDisplay = createSevenSegmentDisplay(49, 48, ballInterval);
}

/* Methode zur Aktualisierung eines Seven-Segment Displays
 * @param display Display welches aktualisiert werden soll
 * @param value der anzuzeigende Wert
 */
void refreshSevenSegmentDisplay(TM1637Display display, int value)
{
  unsigned wait_time_micros = stepperX.nextAction();
  display.showNumberDec(value);
  // Serial.println("Display aktualisiert");
}

/* Methode zur Initialisierung des LCD-Displays
 */
void initializeLcdDisplay()
{
  lcd.init();
  lcd.begin(20, 4);
  lcd.createChar(0, line_left);
  lcd.createChar(1, line_right);
  lcd.createChar(3, ball);
  lcd.createChar(4, empty);
  lcd.createChar(5, arrow);

  lcd.backlight();
}

/* Methode zum Zurücksetzen des Feldes auf dem LCD-Display
 */
void resetLcdField()
{
  lcd.clear();
  int i = 0;
  for (i = 0; i < 4; i++)
  {
    lcd.setCursor(0, i);
    lcd.write(byte(0));
  }
  for (i = 0; i < 4; i++)
  {
    lcd.setCursor(lcdFieldWidth, i);
    lcd.write(byte(1));
  }
  lcd.setCursor(11, 0);
  lcd.print("Ges. ");
  lcd.print("Spin");

  drawBall();
}

/* Methode um den Ball zu zeichnen
 */
void drawBall()
{
  unsigned wait_time_micros = stepperX.nextAction();
  lcd.setCursor(ballPosition[0], ballPosition[1]);
  lcd.write(byte(3));
}

/* Methode um einen Ball zu entfernen
 */
void deleteBall()
{
  unsigned wait_time_micros = stepperX.nextAction();
  lcd.setCursor(ballPosition[0], ballPosition[1]);
  lcd.write(byte(4));
}

/* Methode um Ball 1 und Ball 2 zu zeichnen
 */
void drawBallNumber()
{
  /* Erst Ball 2 zeichnen, falls beide Bälle an derselben Stelle liegen
   * wird dadurch Ball 1 angezeigt
   */
  if (ballTwoValues[0] != 0)
  {
    unsigned wait_time_micros = stepperX.nextAction();
    lcd.setCursor(ballTwoValues[0], ballTwoValues[1]);
    lcd.print("2");
    lcd.setCursor(8, 2);
    lcd.print("2: ");
    lcd.print(ballTwoValues[2]);
    lcd.setCursor(16, 2);
    lcd.print(ballTwoValues[3]);
  }
  if (ballOneValues[0] != 0)
  {
    unsigned wait_time_micros = stepperX.nextAction();
    lcd.setCursor(ballOneValues[0], ballOneValues[1]);
    lcd.print("1");
    lcd.setCursor(8, 1);
    lcd.print("1: ");
    lcd.print(ballOneValues[2]);
    lcd.setCursor(16, 1);
    lcd.print(ballOneValues[3]);
  }
}

//--- Abschnitt: Interrupt Methoden
/* Methode zur Auswertung der Interrupts
 */
void interruptHandler()
{
  if (programmStatus == debug)
  {
    if (handleBallPositionInterrupt)
    {
      {
        if (encBallPosiotionCLKold > digitalRead(encBallPositionCLK))
        {
          ballPositionDT = digitalRead(encBallPositionDT);
        }
      }
      encBallPosiotionCLKold = digitalRead(encBallPositionCLK);
      debugAktion();
      handleBallPositionInterrupt = false;
    }
  }
  else
  {
    if (handleBallPositionInterrupt)
    {
      if (encBallPosiotionCLKold > digitalRead(encBallPositionCLK))
      {
        updateBallPosition(false);
        ballPositionDT = digitalRead(encBallPositionDT);
      }
      encBallPosiotionCLKold = digitalRead(encBallPositionCLK);

      handleBallPositionInterrupt = false;
    }
    if (handleBallSpeedInterrupt)
    {
      updateBallSpeed();
      handleBallSpeedInterrupt = false;
    }
    if (handleBallSpinInterrupt)
    {
      updateBallSpin();
      handleBallSpinInterrupt = false;
    }
    if (handleBallIntervallInterrupt)
    {
      updateBallInterval();
      handleBallIntervallInterrupt = false;
    }
    if (handleLichtschrankeInterrupt)
    {
      lichtschrankeAktion();
      Serial.print("INTERRUPT");
      handleLichtschrankeInterrupt = false;
    }
    if (handleStartInterupt)
    {
      upperESC.arm();
      lowerESC.arm();
      delay(5990);
      handleStartInterupt = false;
    }

    currentDebugPressState = digitalRead(encBallSpinSW);
    if (lastDebugPressState == LOW && currentDebugPressState == HIGH)
    {
      debugReleasedTime = millis();

      Serial.println(debugReleasedTime);
      Serial.println(millis());
    }
    else if (lastDebugPressState == HIGH && currentDebugPressState == LOW)
    {
      debugPressedTime = millis();
      Serial.println("Losgelassen");
    }

    long debugPressDuration = debugReleasedTime - debugPressedTime;
    if (debugPressDuration > LONG_PRESS_TIME)
    {
      Serial.println("Debug aktiviert.");
      // gleich setzen, damit nicht direkt wieder debug aktiviert wird
      debugPressedTime = debugReleasedTime;
      programmStatus = debug;
      for (int i = 0; i < 8; i++)
      {
        oldDebugValues[i] = curDebugValues[i];
      }
    }

    lastDebugPressState = currentDebugPressState;
  }
}

/* Interrupt-Methode für Ballposition
 */
void encoderBallPositionInterrupt()
{
  handleBallPositionInterrupt = true;
}

/* Interrupt-Methode für Ballgeschwindigkeit
 */
void encoderBallSpeedInterrupt()
{
  handleBallSpeedInterrupt = true;
  ballSpeedDT = digitalRead(encBallSpeedDT);
}

/* Interrupt-Methode für Ballspin
 */
void encoderBallSpinInterrupt()
{
  handleBallSpinInterrupt = true;
  ballSpinDT = digitalRead(encBallSpinDT);
}

/* Interrupt-Methode für Ballintervall
 */
void sharedInterrupt()
{ /*// Pin-Deklaration für geshareten 19er
 const byte pinLichtschranke(44)


 const byte encBallIntervalCLK(41);
 const byte encBallIntervalSW(36);
 const byte pinStart(35);
 bool sharedInterArrayOld[5] = {0, 0, 0, 0, 0};
 bool sharedInterArray[5] = {0, 0, 0, 0, 0};*/

  Serial.println(digitalRead(encBallIntervalCLK));
  sharedInterArray[0] = digitalRead(pinLichtschranke);
  sharedInterArray[1] = digitalRead(encBallIntervalCLK);
  sharedInterArray[2] = digitalRead(encBallIntervalSW);
  sharedInterArray[3] = digitalRead(pinStart);
  if (sharedInterArrayOld[0] < sharedInterArray[0])
  {
    handleLichtschrankeInterrupt = true;
  }
  if (sharedInterArrayOld[1] > sharedInterArray[1])
  {
    handleBallIntervallInterrupt = true;
    ballIntervallDT = digitalRead(encBallIntervalDT);
  }
  if (sharedInterArrayOld[2] > sharedInterArray[2])
  {
    // handleIntervalSW=true;
  }
  if (sharedInterArrayOld[3] < sharedInterArray[3])
  {
    handleStartInterupt = true;
  }

  sharedInterArrayOld[0] = sharedInterArray[0];
  sharedInterArrayOld[1] = sharedInterArray[1];
  sharedInterArrayOld[2] = sharedInterArray[2];
  sharedInterArrayOld[3] = sharedInterArray[3];
  sharedInterArrayOld[4] = sharedInterArray[4];
}

/* Interrupt-Methode für Lichtschranke LEGACY
 */
/* void lichtschrankeInterrupt()
{
  if(digitalRead(pinLichtschranke)) {
    handleLichtschrankeInterrupt = true;
  }
}
 */

//--- Abschnitt: Stepper
void stepperXAdjust()
{

  if (ballOneValues[0] == 0)
  {
    xStepper = ((10 + 2 * sollTrommelMan[1]));
  }

  // if (xStepper != 0 )
  // Serial.print(stepperX.getStepsRemaining());
  /*if (stepperX.getStepsRemaining()==0){
     xStepper = xStepper - xStepperLast;


     stepperX.startMove(xStepper);
         xStepperLast = xStepper + xStepperLast;
  }*/
  //
  // else{stepperX.stop();}

  Serial.println(xStepper);
  // Serial.print(Xstepperlast);
  if (xStepperold != xStepper && timer + 200 < millis())
  {
    xStepperold = xStepper;
    timer = millis();

    Wire.beginTransmission(0x55); // Transmit to device with address 85 (0x55)
    Wire.write(xStepper);
    // Wire.write((xStepper));// Sends Potentiometer Reading (8Bit)
    Wire.endTransmission();
  }
}

void stepperRAdjust()
{
  float rSpeed = 60 / 6 / ballInterval;
  // Rspeed = ((60 / ballInterval - (((60 / ballInterval) / 60) * 20000) / 1000 / 2) / 6);
  // Serial.println(Rspeed);
  if (ballThrown)
  {
    stepperR.setSpeedProfile(0, 4000, 10000);
    stepperR.setRPM(rSpeed);
    stepperR.rotate(30);
    ballThrown = false;
    Serial.println("DONE");
  }

  // Serial.println(stepperR.getStepsRemaining());
}

void lichtschrankeAktion()
{
  // TODO: Stepperx durch Steppertrommel ersetzen
  timeBallThrown = millis();
  ballThrown = true;

  handleLichtschrankeInterrupt = false;
  // Zeitpunkt zwischenspeichern + ggf. Position des Motors erfassen
  // Zeitpunkt wird später benötigt um zu prüfen, ob in Zeitraum x inzwischen ein Ball kam
  // Wenn nicht, muss schnell gedreht werden
  // Schwellwert 2 um zu prüfen, ob zwei kammern leer sind -> Ende
  // (Anpassung an das Rastermaß des Revolvers)
}

// --Abschnitt: Debug-Methoden
/* Initialisiserung des Debug-Bildschirmes
 * bei debugParameterStatus initVerHor oder initAbs
 */
void debugInit()
{
  if (debugParameterStatus == initVerHor)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("verticalMin  : ");
    lcd.print(curDebugValues[0]);
    lcd.setCursor(0, 1);
    lcd.print("verticalMax  : ");
    lcd.print(curDebugValues[1]);
    lcd.setCursor(0, 2);
    lcd.print("horizontalMin: ");
    lcd.print(curDebugValues[2]);
    lcd.setCursor(0, 3);
    lcd.print("horizontalMax: ");
    lcd.print(curDebugValues[3]);

    debugParameterStatus = verticalMin;
    debugCursorPos[0] = 19;
    debugCursorPos[1] = 0;
    lcd.setCursor(19, 0);
    lcd.write(byte(5));
  }
  else if (debugParameterStatus == initAbs)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("absSpeedMin  : ");
    lcd.print(curDebugValues[4]);
    lcd.setCursor(0, 1);
    lcd.print("absSpeedMax  : ");
    lcd.print(curDebugValues[5]);
    lcd.setCursor(0, 2);
    lcd.print("absSpinMin   : ");
    lcd.print(curDebugValues[6]);
    lcd.setCursor(0, 3);
    lcd.print("absSpinMax   : ");
    lcd.print(curDebugValues[7]);

    debugParameterStatus = absSpeedMin;
    debugCursorPos[0] = 19;
    debugCursorPos[1] = 0;
    lcd.setCursor(19, 0);
    lcd.write(byte(5));
  }
}

/* Aktualisierung der Debug-Werte bei Interrupt
 */
void debugAktion()
{

  switch (debugParameterStatus)
  {
  case verticalMin:
    debugUpdate(0, 2, 100, 0);
    break;
  case verticalMax:
    debugUpdate(1, 2, 100, 0);
    break;
  case horizontalMin:
    debugUpdate(2, 2, 100, 0);
    break;
  case horizontalMax:
    debugUpdate(3, 2, 100, 0);
    break;
  case absSpeedMin:
    debugUpdate(4, 2, 70, 0);
    break;
  case absSpeedMax:
    debugUpdate(5, 2, 70, 0);
    break;
  case absSpinMin:
    debugUpdate(6, 2, 100, 0);
    break;
  case absSpinMax:
    debugUpdate(7, 2, 100, 0);
    break;
  default:
    break;
  }
}

/* Methode zur aktualisierung der Werte beim Debug
 * @param pos position im Array
 * @param step Schritt um den bei Drehung erhöht wird
 * @param upper Obere Grenze des Wertes
 * @param lower Untere Grenze des Wertes
 */
void debugUpdate(int pos, int step, int upper, int lower)
{
  int csrX = 0;
  // cursorX ist immer 0-3 ab 4 wird von vorne angefangen.
  if (pos < 4)
  {
    csrX = pos;
  }
  else
  {
    csrX = pos - 4;
  }

  curDebugValues[pos] = updatePropertyValue(curDebugValues[pos], step, ballPositionDT);
  curDebugValues[pos] = overflowCorrection(curDebugValues[pos], upper, lower);
  lcd.setCursor(15, csrX);
  lcd.print("   ");
  lcd.setCursor(15, csrX);
  lcd.print(curDebugValues[pos]);
  // Wenn der Wert geändert wurde, wird diese Information gespeichert.
  if (oldDebugValues[pos] != curDebugValues[pos])
  {
    debugValueChanged[pos] = true;
  }
  else
  {
    debugValueChanged[pos] = false;
  }
}

void debugCheckSave()
{
  if (digitalRead(encBallPositionSW) == HIGH && debugValSaved)
  {
    debugValSaved = false;
    lcd.setCursor(debugCursorPos[0], debugCursorPos[1]);
    lcd.write(byte(4));
    if (debugCursorPos[1] + 1 < 4)
    {
      lcd.setCursor(debugCursorPos[0], debugCursorPos[1] + 1);
      lcd.write(empty);
    }
    updateDebugParameterStatus();
    debugCursorPos[1] = debugCursorPos[1] + 1;
  }
  if (digitalRead(encBallPositionSW) == LOW)
  {
    debugValSaved = true;
  }
}

void updateDebugParameterStatus()
{
  switch (debugParameterStatus)
  {
  case initVerHor:
    debugParameterStatus = verticalMin;
    break;
  case verticalMin:
    debugParameterStatus = verticalMax;
    break;
  case verticalMax:
    debugParameterStatus = horizontalMin;
    break;
  case horizontalMin:
    debugParameterStatus = horizontalMax;
    break;
  case horizontalMax:
    debugParameterStatus = initAbs;
    break;
  case initAbs:
    debugParameterStatus = absSpeedMin;
    break;
  case absSpeedMin:
    debugParameterStatus = absSpeedMax;
    break;
  case absSpeedMax:
    debugParameterStatus = absSpinMin;
    break;
  case absSpinMin:
    debugParameterStatus = absSpinMax;
    break;
  case absSpinMax:
    debugParameterStatus = done;
    break;
  default:
    debugParameterStatus = initVerHor;
    break;
  }
  if (debugParameterStatus == done)
  {
    programmStatus = debugDone;
  }
}

int trommelBerechnung(int maxVal, int minVal)
{
}

float absBerechnung(float maxVal, float minVal, float value)
{
  return ((maxVal - minVal) / 100 * value + minVal);
}
void trommelBerechnen()
{ /*int verticalMinVal;
int verticalMaxVal;
int horizontalMinVal;
int horizontalMaxVal;
int absSpeedMinVal;
int absSpeedMaxVal;
int absSpinMinVal;
int absSpinMaxVal;*/

  // int curDebugValues[8] = {0, 100, 0, 100, 0, 100, 0, 100};
  //  MIN u. MAX für L/R + H/T, aktuell 90°
  //  Trommelgeschwindigkeit, absoluter Wert (MIN/MAX)
  //  (MAX-MIN)/100 * x + MIN = 2 * x + MIN = TrommelSpeed
  //  ^^ Schrittberechnung auf Basis von MIN und MAX ^^
  //  Spin von 80:
  //  Trommel1 = Trommelspeed * ((Spin+100)/2)/100 (→80%)
  //  Trommel2 = Trommelspeed * (1-((Spin+100)/2)/100) (→20%)
  //  MAX/MIN Spin analog Trommel
  // Ohne ges. Ball       Vert, Hor, Lower, Upper
  // int sollTromelMan [4]={0,0,0,0};

  sollTrommelMan[0] = absBerechnung(curDebugValues[3], curDebugValues[2], abs(ballPosition[1] - 3));
  sollTrommelMan[1] = absBerechnung(curDebugValues[1], curDebugValues[0], (ballPosition[0] - 1) * 50);
  absSpin[0] = absBerechnung(curDebugValues[7], curDebugValues[6], ballSpin);
  absSpeed[0] = absBerechnung(curDebugValues[5], curDebugValues[4], ballSpeed);
  // Serial.println(absSpin[0],absSpeed[0]);
  Serial.print(sollTrommelMan[1]);
  sollTrommelMan[2] = absSpeed[0] * (1 - ((absSpin[0] + 100) / 2) / 100);
  sollTrommelMan[3] = absSpeed[0] * ((absSpin[0] + 100) / 2) / 100;

  sollTrommelOne[0] = absBerechnung(curDebugValues[3], curDebugValues[2], (ballOneValues[0] - 1) * 50);
  sollTrommelOne[1] = absBerechnung(curDebugValues[1], curDebugValues[0], abs(ballOneValues[1] - 3));
  absSpin[1] = absBerechnung(curDebugValues[8], curDebugValues[7], ballOneValues[3]);
  absSpeed[1] = absBerechnung(curDebugValues[6], curDebugValues[7], ballOneValues[2]);
  sollTrommelOne[2] = absSpeed[1] * (1 - ((absSpin[1] + 100) / 2) / 100);
  sollTrommelOne[3] = absSpeed[1] * ((absSpin[1] + 100) / 2) / 100;

  sollTrommelTwo[0] = absBerechnung(curDebugValues[3], curDebugValues[2], (ballTwoValues[0] - 1) * 50);
  sollTrommelTwo[1] = absBerechnung(curDebugValues[1], curDebugValues[0], abs(ballTwoValues[1] - 3));
  absSpin[2] = absBerechnung(curDebugValues[8], curDebugValues[7], ballTwoValues[3]);
  absSpeed[2] = absBerechnung(curDebugValues[6], curDebugValues[7], ballTwoValues[2]);
  sollTrommelTwo[2] = absSpeed[2] * (1 - ((absSpin[2] + 100) / 2) / 100);
  sollTrommelTwo[3] = absSpeed[2] * ((absSpin[2] + 100) / 2) / 100;
}
void trommelAdjust()
{
  switch (trommelStatus)
  {
  case man:

    // Serial.print(Xstepperlast);
    if (xStepperold != sollTrommelMan[1] && timer + 500 < millis())
    {
      xStepperold = sollTrommelMan[1];
      timer = millis();

      Wire.beginTransmission(0x55); // Transmit to device with address 85 (0x55)
      Wire.write(byte(sollTrommelMan[0]));
      Wire.write(byte(sollTrommelMan[1]));
      Wire.endTransmission();
    }
    sendESC(sollTrommelMan[2], sollTrommelMan[3]);

    break;
  }
}

void sendESC(int upVal, int lowVal)
{
  Serial.println(upVal);
  Serial.println(lowVal);
  upVal = map(upVal, 0, 100, 1000, 2000);
  lowVal = map(lowVal, 0, 100, 1000, 2000);
  lowerESC.speed(lowVal);
  upperESC.speed(upVal);
}
