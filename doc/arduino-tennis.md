- [1. Grundlegende Idee](#1-grundlegende-idee)
- [2. LC-Display](#2-lc-display)
  - [2.1. Normale Ansicht](#21-normale-ansicht)
    - [ToDos](#todos)
  - [2.2. Debug Ansicht](#22-debug-ansicht)
- [3. Ballparameter](#3-ballparameter)
  - [3.1. Position](#31-position)
  - [3.2. Spin](#32-spin)
  - [3.3. Geschwindigkeit](#33-geschwindigkeit)
  - [3.4. Intervall](#34-intervall)
  - [3.5. Methoden](#35-methoden)
- [4. Steppermotoren](#4-steppermotoren)
  - [4.1. induktive Sensoren](#41-induktive-sensoren)
  - [4.2. Steppermotor X](#42-steppermotor-x)
    - [ToDos](#todos-1)
  - [4.3. Steppermotor Y](#43-steppermotor-y)
    - [ToDos](#todos-2)
  - [4.4. Steppermotor R](#44-steppermotor-r)
    - [4.4.1. ToDos](#441-todos)
- [5. Trommeln](#5-trommeln)


## 1. Grundlegende Idee
Eine Maschine, welche aus einer Kammer, zwei Trommeln und einer Lichtschranke besteht.
Das User-Interface ist über Encode, ein LC-Display und Sieben-Segment Anzeigen realisiert.

## 2. LC-Display
### 2.1. Normale Ansicht
Bei der normalen Ansicht soll der Nutzer sehen und einstellen können, an welche Stelle auf dem Spielfeld der Ball gespielt werden soll.
Es sollen zwei Bälle gespeichert werden können.

#### ToDos
- Fehlermeldung; Errorcode im Automatikmodus

### 2.2. Debug Ansicht
Hier könnte Text stehen

## 3. Ballparameter
Der Wurf des Balls durch die Maschine kann über die folgenden Parameter gesteuert werden. Der Wert kann über den dazugehörigen Encoder geändert werden und wird auf der dazugehörigen Sieben-Segment Anzeige angezeigt.
### 3.1. Position
Lorem Ipsum

### 3.2. Spin
- TM1637Display: `ballSpinDisplay`
  - Pins: `51, 50`
- Wert: `ballSpin`
- Boolean
- Encoder-Pins: `CLK: 18 / DT: 46 / SW: 29`

### 3.3. Geschwindigkeit
- TM1637Display: `ballSpeedDisplay`
  - Pins: `53, 52`
- Wert: `ballSpeed`
- Encoder-Pins: `CLK: 18 / DT: 46 / SW: 29`
  
### 3.4. Intervall
- TM1637Display: `ballIntervalDisplay`
  - Pins: `49, 48`
- Wert: `ballSpin`
- Encoder-Pins: `CLK: 19 / DT: 47 / SW: undefined` (Shared Interrupt)

### 3.5. Methoden
- `int updatePropertyValue(int value, int step, boolean increase)`
- `int overflowCorrection(int value, int upper, int lower)`

## 4. Steppermotoren
### 4.1. induktive Sensoren
### 4.2. Steppermotor X 
Der Steppermotor X steuert die horizontale Richtung des Balls. Dies erfolgt, indem die beiden Trommeln um einen errechneten Betrag horizontal gedreht werden.
Hierbei sind folgende Modi zu beachten:
1. Manueller Modus
   Es sind keine Bälle gespeichert, die horizontale Richtung des Balls ist analog des auf dem LC-Display dargestellten Balls.
2. Automatikmodus
   Es sind zwei Bälle gespeichert, die horizontale Richtung der Trommeln wechselt zwischen den eingespeicherten Werten.

Die Anpassung der horizontalen Richtung erfolgt permanent innterhalb des Loops über die Methode `stepperXAdjust`.

#### ToDos
- Nullung des Motors durch den induktiven Sensor
- Wechsel L/R im Automatikmodus
- Drehung nach Auswurf über Lichtschranke

### 4.3. Steppermotor Y
Der Steppermotor Y steuert die vertikale Richtung des Balls. Dies erfolgt, indem die beiden Trommeln um einen errechneten Betrag vertikal geneigt werden.
Hierbei sind folgende Modi zu beachten:
1. Manueller Modus
   Es sind keine Bälle gespeichert, die vertikale Richtung des Balls ist analog des auf dem LC-Display dargestellten Balls.
2. Automatikmodus
   Es sind zwei Bälle gespeichert, die vertikale Richtung der Trommeln wechselt zwischen den eingespeicherten Werten.

Die Anpassung der vertikalen Richtung erfolgt permanent innterhalb des Loops über die Methode `stepperYAdjust`.

#### ToDos
Ja Hans, analog Steppermotor X

### 4.4. Steppermotor R
Der Steppermotor R regelt die Ballzufuhr aus dem Revolver zu den Trommeln.
Die Geschwindigkeit ist abhängig von dem eingestellten [Intervall des Balls](#34-intervall) und wird wie folgt berechnet: `rSpeed = 60 / 6 / ballInterval`

Die Anpassung der Geschwindigkeit erfolgt permanent innerhalb des Loops über die Methode `stepperRAdjust`.

#### 4.4.1. ToDos
- Geschwindigkeit einmalig ermitteln, bei Anpassung des Intervalls; diese dann halten
- Außer Overdrive aktiv: Auf Basis der Lichtschranke Geschwindkeit erhöhen (es kam seit Zeitraum x kein Ball mehr -> schneller drehen)
- Wenn wieder im Normallauf, alte Geschwinidgkeit einstellen
- Hausaufgabe für Eric: Lichtschranke ja oder nein

## 5. Trommeln
Text schreiben, basierend auf Geschwindikeit und Spin