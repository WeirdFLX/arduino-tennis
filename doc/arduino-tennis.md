- [Grundlegende Idee](#grundlegende-idee)
- [LC-Display](#lc-display)
  - [Normale Ansicht](#normale-ansicht)
  - [Debug Ansicht](#debug-ansicht)
- [Ballparameter](#ballparameter)
  - [Position](#position)
  - [Spin](#spin)
  - [Geschwindigkeit](#geschwindigkeit)
  - [Intervall](#intervall)
  - [Methoden](#methoden)
- [Steppermotoren](#steppermotoren)
  - [induktive Sensoren](#induktive-sensoren)
  - [Steppermotor X](#steppermotor-x)
  - [Steppermotor Y](#steppermotor-y)
  - [Steppermotor R](#steppermotor-r)
- [Trommeln](#trommeln)


## Grundlegende Idee
Eine Maschine, welche aus einer Kammer, zwei Trommeln und einer Lichtschranke besteht.
Das User-Interface ist über Encode, ein LC-Display und Sieben-Segment Anzeigen realisiert.

## LC-Display
### Normale Ansicht
Bei der normalen Ansicht soll der Nutzer sehen und einstellen können, an welche Stelle auf dem Spielfeld der Ball gespielt werden soll.
Es sollen zwei Bälle gespeichert werden können.

### Debug Ansicht


## Ballparameter
Der Wurf des Balls durch die Maschine kann über die folgenden Parameter gesteuert werden. Der Wert kann über den dazugehörigen Encoder geändert werden und wird auf der dazugehörigen Sieben-Segment Anzeige angezeigt.
### Position
Lorem Ipsum

### Spin
- TM1637Display: `ballSpinDisplay`
  - Pins: `51, 50`
- Wert: `ballSpin`
- Boolean
- Encoder-Pins: `CLK: 18 / DT: 46 / SW: 29`

### Geschwindigkeit
- TM1637Display: `ballSpeedDisplay`
  - Pins: `53, 52`
- Wert: `ballSpeed`
- Encoder-Pins: `CLK: 18 / DT: 46 / SW: 29`
  
### Intervall
- TM1637Display: `ballIntervalDisplay`
  - Pins: `49, 48`
- Wert: `ballSpin`
- Encoder-Pins: `CLK: 19 / DT: 47 / SW: undefined` (Shared Interrupt)

### Methoden
- `int updatePropertyValue(int value, int step, boolean increase)`
- `int overflowCorrection(int value, int upper, int lower)`

## Steppermotoren
### induktive Sensoren
### Steppermotor X 
Horizontale position des Balls

### Steppermotor Y
Vertikale Position des Balls

### Steppermotor R
Der Steppermotor R regelt die Ballzufuhr aus dem Revolver zu den Trommeln.
Die Geschwindigkeit ist abhängig von dem eingestellten [Intervall des Balls](#intervall) und wird wie folgt berechnet: `rSpeed = 60 / 6 / ballInterval`

Die Anpassung der Geschwindigkeit 

Ballzufuhr; Lichtschranke 

## Trommeln