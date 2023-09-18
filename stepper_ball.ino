#include <A4988.h>
#include <BasicStepperDriver.h>
#include <DRV8825.h>
#include <DRV8834.h>
#include <DRV8880.h>
#include <MultiDriver.h>
#include <SyncDriver.h>

#include <Wire.h>

#include <A4988.h>

#define MOTOR_STEPS 800
// Microstepping mode. If you hardwired it to save pins, set to the same value here.
#define MICROSTEPS 1

#define DIR 7
#define STEP 6

// #define SLEEP 13 // optional (just delete SLEEP from everywhere if not used)

/*
 * Choose one of the sections below that match your board
 */

#include "DRV8834.h"
#define M0 10
#define M1 11
DRV8834 stepperX(MOTOR_STEPS, DIR, STEP, M0, M1);
unsigned long xStepperLast;
int RxByte;
long xsollStepper;
long XStepper;
long timer;
volatile byte receivedData[2];
boolean newDataReceived = false;

void I2C_RxHandler(int numBytes)
{
    while (Wire.available())
    { // Read Any Received Data
        xsollStepper = (Wire.read() - 110) * 200;
        // yStepper = Wire.read(2);  }
    }
    Serial.println(xsollStepper);
}
void setup()
{
    A4988(short(200), short(7), short(6));
    stepperX.setSpeedProfile(BasicStepperDriver::LINEAR_SPEED, 10000, 10000);
    // Serial.println(x);
    stepperX.setRPM(1500);
    Serial.begin(9600);
    Wire.begin(0x55); // Initialize I2C (Slave Mode: address=0x55 )
    Wire.onReceive(receiveData(2));

    //Wire.onReceive(I2C_RxHandler);
}

void loop()
{
    unsigned wait_time_micros = stepperX.nextAction();
    if (stepperX.getStepsRemaining() == 0)
    {

        XStepper = xsollStepper - xStepperLast;

        xStepperLast = XStepper + xStepperLast;

        stepperX.startMove(XStepper);
    }
    if (newDataReceived) {
        x = receivedData[0];
        y = receivedData[1];
        newDataReceived = false;
    }
}

void receiveData(int amount){
    for(int i= 0; i < amount; i++) 
    {
        receivedData[i] = Wire.read();
    }
    newDataReceived = true;
}