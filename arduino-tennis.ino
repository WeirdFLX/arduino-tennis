#include <LiquidCrystal_I2C.h>
#include <TM1637Display.h>
#include <BasicStepperDriver.h>
#include <EEPROM.h>

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
byte empty[8] = {B00000, B00000, B00000, B00000, B00000, B00000, B00000, B00000};
// Icon Arrow
byte arrow[8] = {B00000, B00110, B01100, B11111, B11111, B01100, B00110, B00000};
// Breite des Feldes
int lcdFieldWidth = 6;

// Steppersettings
int stepsprorev(200);
int revspeed;
// Variablen für Stepper
BasicStepperDriver stepperX(stepsprorev, 23, 22);
BasicStepperDriver stepperY(stepsprorev, 26, 27);
BasicStepperDriver stepperR(stepsprorev, 24, 25);
// Stepperlogging
unsigned long Xstepper;
unsigned long Xstepperlast;
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

// Variablen und Display für Ballgeschwindigkeit
TM1637Display ballSpeedDisplay = TM1637Display(53, 52);
int ballSpeed;
bool handleBallSpeedInterrupt;
bool ballSpeedDT;
// Pin-Deklarationen
const byte encBallSpeedCLK(3);
const byte encBallSpeedDT(45);

// Variablen und Display für Ballspin
TM1637Display ballSpinDisplay = TM1637Display(51, 50);
int ballSpin;
bool handleBallSpinInterrupt;
bool ballSpinDT;
// Pin-Deklarationen
const byte encBallSpinCLK(18);
const byte encBallSpinDT(46);
// für Debug-Modus
const byte encBallSpinSW(29);

// Variablen und Display für Ballintervall
TM1637Display ballIntervalDisplay = TM1637Display(49, 48);
int ballInterval;
bool handleBallIntervallInterrupt;
bool ballIntervallDT;
// Pin-Deklarationen
const byte encBallIntervalCLK(19);
const byte encBallIntervalDT(47);

// Pin-Deklaration für geshareten 19er
const byte pinBallInterval(42);
const byte pinLichtschranke(44);
bool sharedInterArrayOld[2] = {0, 0};
bool sharedInterArray[2] = {0, 0};


// Variablen für Lichtschranke
bool handleLichtschrankeInterrupt;
// Pin-Deklaration
//const byte lichtschranke(19);

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

int curDebugValues[8] = {0, 100, 25, 75, 0, 100, 0, 100};
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

// Setup-Methode
void setup()
{
  // Serielle Ausgabe verbinden für ggf. Debugging
  Serial.begin(9600);

  Xstepperlast = 0;
  // Schrittmotoren Parameter
  revspeed = 30;
  stepperX.begin(revspeed, 1);
  stepperY.begin(revspeed, 1);
  stepperR.begin(revspeed, 1);
  Serial.println(stepperR.getSteps());
  stepperX.move(2);

  // Pins setzen und Interrupts anbinden
  pinMode(encBallPositionDT, INPUT);
  pinMode(encBallPositionSW, INPUT);
  pinMode(encBallSpeedDT, INPUT);
  pinMode(encBallSpinDT, INPUT);
  pinMode(encBallSpinSW, INPUT);
  pinMode(encBallIntervalDT, INPUT);
  pinMode(pinBallInterval, INPUT);
  pinMode(pinLichtschranke, INPUT);
  //pinMode(lichtschranke, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encBallPositionCLK), encoderBallPositionInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(encBallSpeedCLK), encoderBallSpeedInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(encBallSpinCLK), encoderBallSpinInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(encBallIntervalCLK), sharedInterrupt, CHANGE);
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
}

// Main-Loop
void loop()
{
  interruptHandler();
  trommelBerechnen();
  if (programmStatus == debug)
  {
    debugInit();
    debugCheckSave();
    // MIN u. MAX für L/R + H/T, aktuell 90°
    // Trommelgeschwindigkeit, absoluter Wert (MIN/MAX)
    // (MAX-MIN)/100 * x + MIN = 2 * x + MIN = TrommelSpeed
    // ^^ Schrittberechnung auf Basis von MIN und MAX ^^
    // Spin von 80:
    // Trommel1 = Trommelspeed * ((Spin+100)/2)/100 (→80%)
    // Trommel2 = Trommelspeed * (1-((Spin+100)/2)/100) (→20%)
    // MAX/MIN Spin analog Trommel
  }
  else if (programmStatus == debugDone)
  {
    // Speichern der geänderten Werte, nur falls geändert
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
    stepperXadjust();
    stepperRadjust();
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
}

//--- Abschnitt: Balleigenschaften aktualisieren
/* Methode um die Position des Balles zu aktualisieren
 * @param overrideDT ballPositionDT wird ignoriert wenn der Wert mit true gefüllt ist, es wird um 1 erhöht.
 */
void updateBallPosition(bool overrideDT)
{
  deleteBall();
  drawBallNumber();
  if (overrideDT)
  {
    ballPosition[0] = updatePropertyValue(ballPosition[0], 1, overrideDT);
  }
  else
  {
    ballPosition[0] = updatePropertyValue(ballPosition[0], 1, ballPositionDT);
  }
  wrapBallPosition();
  drawBall();
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
  int oldBallSpeed = ballSpeed;
  ballSpeed = updatePropertyValue(ballSpeed, 10, ballSpeedDT);
  ballSpeed = overflowCorrection(ballSpeed, 100, 10);
  if (oldBallSpeed != ballSpeed)
  {
    refreshSevenSegmentDisplay(ballSpeedDisplay, ballSpeed);
  }
}

/* Methode zur aktualisierung des Ball-Spins,
 * sowie des dazugehörigen Displays
 */
void updateBallSpin()
{
  int oldBallSpin = ballSpin;
  ballSpin = updatePropertyValue(ballSpin, 10, ballSpinDT);
  ballSpin = overflowCorrection(ballSpin, 100, -100);
  if (oldBallSpin != ballSpin)
  {
    refreshSevenSegmentDisplay(ballSpinDisplay, ballSpin);
  }
}

/* Methode zur aktualisierung des Ball-Intervalls,
 * sowie des dazugehörigen Displays
 */
void updateBallInterval()
{
  int oldBallInterval = ballInterval;
  ballInterval = updatePropertyValue(ballInterval, 1, ballIntervallDT);
  ballInterval = overflowCorrection(ballInterval, 20, 2);
  if (oldBallInterval != ballInterval)
  {
    refreshSevenSegmentDisplay(ballIntervalDisplay, ballInterval);
  }
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
    return value - step;
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
  display.showNumberDec(value);
  Serial.println("Display aktualisiert");
}

/* Methode zur Initialisierung des LCD-Displays
 */
void initializeLcdDisplay()
{
  lcd.begin(20, 4);
  lcd.createChar(0, line_left);
  lcd.createChar(1, line_right);
  lcd.createChar(3, ball);
  lcd.createChar(4, empty);
  lcd.createChar(5, arrow);
  lcd.init();
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
  lcd.setCursor(ballPosition[0], ballPosition[1]);
  lcd.write(byte(3));
}

/* Methode um einen Ball zu entfernen
 */
void deleteBall()
{
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
      debugAktion();
      handleBallPositionInterrupt = false;
    }
  }
  else
  {
    if (handleBallPositionInterrupt)
    {
      updateBallPosition(false);
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

    currentDebugPressState = digitalRead(encBallSpinSW);
    if (lastDebugPressState == HIGH && currentDebugPressState == LOW)
    {
      debugReleasedTime = millis();
      Serial.println(debugReleasedTime);
      Serial.println(millis());
    }
    else if (lastDebugPressState == LOW && currentDebugPressState == HIGH)
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
  ballPositionDT = digitalRead(encBallPositionDT);
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
{
    //Serial.println(digitalRead(pinLichtschranke));
  sharedInterArray[0] = digitalRead(pinLichtschranke);  
  sharedInterArray[1] = digitalRead(pinBallInterval);

  if (sharedInterArrayOld[0] < sharedInterArray[0])
  {
    handleLichtschrankeInterrupt = true;
  }
  else if (sharedInterArrayOld[1] > sharedInterArray[1])
  {
    handleBallIntervallInterrupt = true;
    ballIntervallDT = digitalRead(encBallIntervalDT);
  }

  sharedInterArrayOld[0] = sharedInterArray[0];
  sharedInterArrayOld[1] = sharedInterArray[1];  
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
void stepperXadjust()
{
  // Serial.println(x);

  if (ballOneValues[0] == 0)
  {
    Xstepper = ((sollTrommelMan[1] * 1) - 50);
  }
  Xstepper = Xstepper - Xstepperlast;
  Xstepperlast = Xstepper + Xstepperlast;
  if (Xstepper != 0)
  {
    stepperX.move(Xstepper);
  }

  // Serial.println(Xstepper);
  // Serial.print(Xstepperlast);
}

void stepperRadjust()
{
  float Rspeed = 60 / 6 / ballInterval;
  //Rspeed = ((60 / ballInterval - (((60 / ballInterval) / 60) * 20000) / 1000 / 2) / 6);
  //Serial.println(Rspeed);
  if(ballThrown) {
    stepperR.setSpeedProfile(0, 1000, 5000);
    stepperR.setRPM(Rspeed);
    stepperR.rotate(30);
    ballThrown = false;

  }

  //Serial.println(stepperR.getStepsRemaining());

}

void lichtschrankeAktion()
{
  // TODO: Stepperx durch Steppertrommel ersetzen
  timeBallThrown = millis();
  stepperR.stop();
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
    debugUpdate(4, 2, 100, 0);
    break;
  case absSpeedMax:
    debugUpdate(5, 2, 100, 0);
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
      lcd.write(byte(5));
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

int axisBerechnung(int maxVal, int minVal)
{
}

float trommelBerechnung(float maxVal, float minVal, float value)
{
  return ((maxVal - minVal) / 100 * value + minVal);
}
void trommelBerechnen()
{
  // Ohne ges. Ball       Vert, Hor, Lower, Upper
  // int sollTromelMan [4]={0,0,0,0};
  switch (trommelStatus)
  {
  case man:
    sollTrommelMan[1] = trommelBerechnung(curDebugValues[3], curDebugValues[2], (ballPosition[0] - 1) * 25);
    sollTrommelMan[0] = trommelBerechnung(curDebugValues[1], curDebugValues[0], abs(ballPosition[0] - 3));
    // Serial.println(sollTrommelMan[1]);
    break;
  case one:
    break;
  case two:
    break;
  default:
    break;
  }
}