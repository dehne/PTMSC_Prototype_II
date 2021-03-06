/****
 * 
 * This file is a portion of the package FlyingPlatform, a library that 
 * provides an Arduino sketch with the ability to control four stepper motors 
 * that operate a "flying camera" setup in which a platform is connected by 
 * cables to four stepper-motor-controlled winches. By judiciously reeling in 
 * and letting out the cables, the platform can be moved around in three 
 * space.
 * 
 * See FlyingPlatform.h for details.
 *
 *****
 * 
 * FlyingPlatform V0.4, October 2020
 * Copyright (C) 2020 D.L. Ehnebuske
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 * 
 ****/

#include "FlyingPlatform.h"
#include <util/atomic.h>


/**
 * 
 * Variables held in common with the ISRs. I don't like this, but I can find no 
 * clean way of sharing variables with ISRs.
 * 
 **/
volatile static bool nextReady;             // Lock for sharing nextXxxx variables. true => ISR owns false => run()
static bool nextShortening[4];              // Buffer for Whether the cable is shortening or lengthening in next batch
static long nextPendingSteps[4];            // Buffer for pendingSteps in next batch of steps
static unsigned long nextDsInterval[4];     // Buffer for interval between steps (μs) in next batch

volatile static bool timerISRHasWork;       // True if timer ISR still has steps to take. (Owned by ISR)
volatile static long cableSteps[4];         // The current length of the cables, in steps (Owned by ISR; use ATOMIC access)
static byte dirPin[4];                      // Direction digital output pins. High means lengthen cables (Owned by object)
static byte stepPin[4];                     // Step digital output pins. Normally LOW. Pulse HIGH to step. (Owned by object)
volatile static bool isEnabled;             // Whether motor drivers are enabled
volatile static bool isCalibrated;          // True if we know where the platform is
volatile bool isCalibrating;                // True if calibration is underway
enum calStates : byte {cal_start, cal_goingRight, cal_goingLeft, cal_aBitMore, cal_done};
volatile calStates state[4];                // Current state of the state machines
volatile int aBitMore[4];                   // The number of additional steps taken in state cal_aBitMore ()
static byte calClkPin;                      // The calReader clock pin
static byte calDataPin;                     // The calReader data pin
static int calData;                         // The data received from calReader (Owned by calClkISR)

#ifdef FP_DEBUG_ISR
volatile static bool captured = false;
volatile static bool printed = false;
volatile static unsigned long tcurMicros;
volatile static unsigned long tTargetMicros0;
volatile static unsigned long tDsInterval0;
#endif


/**
 * 
 * Timer interrupt service routine. Invoked whenever the count in Timer 2 
 * reaches the value specified in register OCR2A. Because Timer 2 is set up 
 * (in begin()) to tick once every 2μs and OCR2A is an 8-bit register, the 
 * longest we can go between interrupts is 512μs. We adjust OCR2A as needed 
 * to cause interrupts at shorter intervals.
 * 
 * Basically, when an interrupt occurs, after checking to see if there's 
 * anything at all to do, we check to see if it's time to load a new batch of 
 * steps. If so, we copy the batch data from the nextXxxx variables to the
 * live ones and reset the nextReady lock to indicate we did so. In the 
 * process we also set the direction pin on each motor so that steps taken 
 * during the batch will be in the correct direction.
 * 
 * Next we dispatch any steps whose time has come. Dispatching a step consists 
 * of emiting a HIGH pulse on the appropriate GPIO pin. In the process we also 
 * figure out if there are any steps whose time will become due during the 
 * next ms, and, if so, what micros() for it will be when it's due.
 * 
 * Finally, if there is a step whose time will come due inside the next 512μs, 
 * we adjust the value of OCR2A so that an interrupt will occur at the correct 
 * time. If there's isn't one, we set OCR2A to 0xFF to cause the next 
 * interrupt to happen in 512μs.
 * 
 **/
ISR(TIMER2_COMPA_vect) {
    static byte nFinished = 4;                              // Number of motors finished with this batch
    static long pendingSteps[4];                            // Number of remaining steps by which to change each cable
    static bool shortening[4];                              // True if shortening the corresponding cable
    static unsigned long dsInterval[4];                     // Interval between each cable's steps (μs)
    static unsigned long targetMicros[4] =  {0, 0, 0, 0};   // When to dispatch next step for each cable

    if (nFinished == 4 && !nextReady) {                     // No work if finished with batch but no new batch ready
        timerISRHasWork = false;
        return;
    }
    unsigned long curMicros = micros();
    if (nFinished == 4) {                       // Finished with batch (and new batch ready)
        // Copy batch data from nextXxx variables to ISR's variables; set rotation directions
        for (byte i = 0; i < 4; i++) {
            pendingSteps[i] = nextPendingSteps[i];
            shortening[i] = nextShortening[i];
            if (shortening[i]) {
                digitalWrite(dirPin[i], FP_SHORTEN);   // Set to shorten cable
            } else {
                digitalWrite(dirPin[i], FP_LENGTHEN);  // Set to lengthen cable
            }
            dsInterval[i] = nextDsInterval[i];
            if (curMicros > targetMicros[i]) {
                targetMicros[i] = curMicros;
            }
        }
        #ifdef FP_DEBUG_ISR
        if (!captured) {
            tTargetMicros0 = targetMicros[0];
            tcurMicros = curMicros;
            tDsInterval0 = dsInterval[0];
            captured = true;
        }
        #endif
        nextReady = false;                  // Hand nextXxx variables back to object
    }

    // Process any steps that are ready. Along the way, discover micros() at 
    // which we'll need to process the next step(s). FP_MAX_JITTER needs to be 
    // big enough so that there is time to get through dispatching these steps 
    // and set OCR0A and exit before shortestT passes by or we'll miss a whole 
    // ms.
    nFinished = 0;
    unsigned long shortestT = curMicros + 1000;         // Assume no steps in next ms
    for (byte i = 0; i < 4; i++) {
        if (pendingSteps[i] > 0 && curMicros >= targetMicros[i] - FP_MAX_JITTER) {
            digitalWrite(stepPin[i], HIGH);
            digitalWrite(stepPin[i], LOW);
            cableSteps[i] += shortening[i] ? -1 : 1;
            pendingSteps[i]--;
            targetMicros[i] += dsInterval[i];
        }
        if (pendingSteps[i] == 0) {
            nFinished++;
        } else if (targetMicros[i] < shortestT) {
            shortestT = targetMicros[i];
        }
    }

    timerISRHasWork = nFinished != 4 || nextReady; // Indicate whether we still have work
    #ifdef FP_DEBUG_MTR
    digitalWrite(A5, timerISRHasWork ? HIGH : LOW);
    #endif


    // shortestT now holds the micros() at which the next step(s) should be 
    // taken. Figure out what OCR2A should be to interrupt appropriately. If
    // it's less than 512μs, set it to half that since the timer counts by 2μs. 
    // Otherwise set it to 0xFF.
    unsigned long microsToGo = (shortestT - curMicros);
    if (microsToGo < 512) {
        OCR2A = (microsToGo >> 1) & 0xFF;
    } else {
        OCR2A = 0xFF;
    }
}

/**
 * 
 * Interrupt service routine used during calibration. This ISR is attached to 
 * CAL_CLK_PIN when it's time to calibrate. It steps the motors to move the 
 * platform to the position defined by the home photo interrupters on each 
 * cable winch. Once the position is reached, the ISR detaches itself. See 
 * calReader sketch for details.
 * 
 * To do a calibration
 *      1. Only do this if isCalibrating == false
 *      2. Attach the ISR: attachInterrupt(digitalPinToInterrupt(calClkPin), calClkISR, RISING);
 *      3. Don't try to run the motors while isCalibrating is true
 * 
 * To stop a calibration already in progress
 *      1. Detach the interrupt: detachInterrupt(digitalPinToInterrupt(calClkPin));
 *      2. Set isCalibrating = false;
 * 
 **/
void calClkISR() {
  if (!isCalibrating) {
    isCalibrating = true;
    isCalibrated = false;
    state[0] = state[1] = state[2] = state[3] = cal_start;
  }
  calData = (calData << 1) | digitalRead(calDataPin);
  if ((calData & FP_CAL_SYNC_MASK) == FP_CAL_SYNC) {
    byte doneCount = 0;
    for (byte cable = 0; cable < 4; cable++) {
      // calData: 0bxxxxxxx11110abcd where a is bit for cable 0; b is for 1; ...
      byte reading = (calData >> (3 - cable) & 1);
      switch (state[cable]) {
        case cal_start:
          aBitMore[cable] = 0;
          if (reading == FP_CAL_ROC) {
            state[cable] = cal_goingLeft;
          } else {
            state[cable] = cal_goingRight;
          }
          break;
        case cal_goingRight:
          if (reading == FP_CAL_ROC) {
            state[cable] = cal_done;
          } else {
            digitalWrite(dirPin[cable], FP_SHORTEN);
            digitalWrite(stepPin[cable], HIGH);
            digitalWrite(stepPin[cable], LOW);
          }
          break;
        case cal_goingLeft:
          if (reading == FP_CAL_LOC) {
            state[cable] = cal_aBitMore;  
          } else {
            digitalWrite(dirPin[cable], FP_LENGTHEN);
            digitalWrite(stepPin[cable], HIGH);
            digitalWrite(stepPin[cable], LOW);
          }
          break;
        case cal_aBitMore:
          if (++aBitMore[cable] >= FP_CAL_MORE) {
            state[cable] = cal_goingRight;
          } else {
            digitalWrite(dirPin[cable], FP_LENGTHEN);
            digitalWrite(stepPin[cable], HIGH);
            digitalWrite(stepPin[cable], LOW);
          }
          break;
        case cal_done:
          doneCount++;
          break;
      }
    }
    if (doneCount == 4) {
      detachInterrupt(digitalPinToInterrupt(calClkPin));
      isCalibrating = false;
      isCalibrated = true;
    }
  }
    return;
}

FlyingPlatform::FlyingPlatform(
    byte pin0D, byte pin0P, 
    byte pin1D, byte pin1P, 
    byte pin2D, byte pin2P, 
    byte pin3D, byte pin3P,
    byte pinEn,
    byte pinCC, byte pinCD,
    long spaceWidth, long spaceDepth, long spaceHeight) {
        dirPin[0] = pin0D;
        dirPin[1] = pin1D;
        dirPin[2] = pin2D;
        dirPin[3] = pin3D;
        stepPin[0] = pin0P;
        stepPin[1] = pin1P;
        stepPin[2] = pin2P;
        stepPin[3] = pin3P;
        enablePin = pinEn;
        calClkPin = pinCC;
        calDataPin = pinCD;
        space = fp_Point3D {spaceWidth, spaceDepth, spaceHeight};
        marginsMin = fp_Point3D {0, 0, 0};
        marginsMax = fp_Point3D {spaceWidth, spaceDepth, spaceHeight};
        targetsMin = fp_Point3D {marginsMin.x + FP_TARGETS_BUFFER, marginsMin.y + FP_TARGETS_BUFFER, marginsMin.z + FP_TARGETS_BUFFER};
        targetsMax = fp_Point3D {marginsMax.x - FP_TARGETS_BUFFER, marginsMax.y - FP_TARGETS_BUFFER, marginsMax.z + FP_TARGETS_BUFFER};
    }

void FlyingPlatform::begin() {
    pinMode(enablePin, OUTPUT);
    for (byte i = 0; i < 4; i++) {
        pinMode(dirPin[i], OUTPUT);
        pinMode(stepPin[i], OUTPUT);
        cableSteps[i] = 0;          // timer ISR not yet running; no ATOMIC stuff needed
        nextPendingSteps[i] = 0;
        nextDsInterval[i] = 0;
        nextShortening[i] = false;
    }
    pinMode(calClkPin, INPUT);
    pinMode(calDataPin, INPUT);
    isCalibrated = false;
    isCalibrating = false;
    calData = 0;
    digitalWrite(enablePin, LOW);   // Enable (active LOW) motor drivers
    isEnabled = true;

    batchSteps = FP_BATCH_STEPS;
    t = 1.0;
    newMove = false;
    stopping = false;
    maxSpeed = FP_MAX_SPEED;
    hHeading = 0;
    hChange = fp_straight;
    vHeading = FP_LEVEL;
    vChange = fp_level;
    turnMicros = 0;
    #if defined(FP_DEBUG_LED) || defined(FP_DEBUG_MTR)
    pinMode(A5, OUTPUT);
    digitalWrite(A5, LOW);
    #endif

    // Set up our Timer 2 ISR
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        TCCR2A = 0x00;
        TCCR2B = 0x00;                  // Stop Timer 2
        OCR2A = 0xFF;                   // Set number of clock ticks until interrupt.
        TCCR2A = _BV(WGM21);            // WGM2[2:0] = 0x2 i.e., Set "Clear Timer on Compare" (CTC) mode
        TCCR2B = _BV(CS21) | _BV(CS20); // CS2[2:0] = 0x3  i.e., Clock/64 (From prescaler)
        TIMSK2 |= _BV(OCIE2A);          // Turn on OCR2A compare interrupts
    }
}

bool FlyingPlatform::run() {
    #ifdef FP_DEBUG_RU
    static byte newMoveCount = 0;
    #endif
    #ifdef FP_DEBUG_ISR
        if (captured && !printed) {
            unsigned long ttm, tcm, tdi;
            ATOMIC_BLOCK(ATOMIC_FORCEON) {
                ttm = tTargetMicros0;
                tcm = tcurMicros;
                tdi = tDsInterval0;
            }
            Serial.print(F("ISR Apparent new move. currentMicros: "));
            Serial.print(tcm);
            Serial.print(F(", targetMicros[0]: "));
            Serial.print(ttm);
            Serial.print(F(", dsInterval[0]: "));
            Serial.println(tdi);
            printed = true;
        }
    #endif
    
    if (!isCalibrated || !isEnabled) {
        return false;
    }
    unsigned long curMicros = micros();

    // Deal with turning
    if (curMicros - turnMicros >= FP_TURN_INTERVAL) {
        bool changed = false;
        if (hChange == fp_left) {
            hHeading = (hHeading + 1) % FP_N_HHEADINGS;
            changed = true;
        } else if (hChange == fp_right) {
            hHeading = (hHeading + FP_N_HHEADINGS - 1) % FP_N_HHEADINGS;
            changed = true;
        }
        switch (vChange) {
            case fp_falling:
                if (vHeading > 0) {
                    vHeading--;
                    changed = true;
                }
                break;
            case fp_level:
                if (vHeading > FP_LEVEL) {
                    vHeading--;
                    changed = true;
                } else if (vHeading < FP_LEVEL) {
                    vHeading++;
                    changed = true;
                }
                break;
            case fp_rising:
                if (vHeading < FP_N_VHEADINGS - 1) {
                    vHeading++;
                    changed = true;
                }
                break;
        }
        // If there's a new heading and we're going and we're not trying to stop
        if (changed && timerISRHasWork && !stopping) {
            moveTo(newTarget());
        }
        turnMicros = curMicros;
    }

    // If we're stopping, no further work until the queued ISR work finishes
    if (stopping && timerISRHasWork) {
        return true;
    }


    #ifdef FP_DEBUG_LED
    digitalWrite(A5, HIGH); // Turn on light at A5
    #endif

    // If we're stopping (and the queued ISR work is finished), we're stopped.
    if (stopping) {
        #ifdef FP_DEBUG_GEO
        fp_CableBundle cs;
        for (byte i = 0; i < 4; i++) {      // ISR is idle; no need for atomic access
            cs.c[i] = cableSteps[i];
        }
        fp_Point3D here = cbToP3D(cs);
        if (here.x < marginsMin.x || here.y < marginsMin.y || here.z < marginsMin.z ||
          here.x > marginsMax.x || here.y > marginsMax.y || here.z > marginsMax.z) {
            Serial.print(F("run() Stopped out of bounds: (<"));
            Serial.print(marginsMin.x);
            Serial.print(F("> "));
            Serial.print(here.x);
            Serial.print(F(" <"));
            Serial.print(marginsMax.x);
            Serial.print(F(">, <"));
            Serial.print(marginsMin.y);
            Serial.print(F("> "));
            Serial.print(here.y);
            Serial.print(F(" <"));
            Serial.print(marginsMax.y);
            Serial.print(F(">, <"));
            Serial.print(marginsMin.z);
            Serial.print(F("> "));
            Serial.print(here.z);
            Serial.print(F(" <"));
            Serial.print(marginsMax.z);
            Serial.print(F(">), cables: ("));
            for (byte i = 0; i < 4; i++) {
                Serial.print(cs.c[i]);
                Serial.print( i == 3 ? F(").\n") : F(", "));
            }
        }
        #endif
        stopping = false;
        t = 1.0;            // Move is done
        newMove = false;    // And we're not starting another
        return false;
    }

    // If all done with move and nothing new has been scheduled, no use doing more work.
    if (t == 1.0 && !newMove) {

        #ifdef FP_DEBUG_LED
        digitalWrite(A5, LOW);
        #endif

        return false;
    }

    // If it's time to calculate the next batch of steps
    if (!nextReady) {
        // If we're to go off in a new direction, set up for first batch of the move from wherever we are to target
        if (newMove) {            
            newMove = false;
            source = where();
            float moveLength = sqrt(
                (target.x - source.x) * (target.x - source.x) + 
                (target.y - source.y) * (target.y - source.y) + 
                (target.z - source.z) * (target.z - source.z));
            #ifdef FP_DEBUG_GEO
            if (source.x < marginsMin.x || source.y < marginsMin.y || source.z < marginsMin.z ||
                source.x > marginsMax.x || source.y > marginsMax.y || source.z > marginsMax.z) {
                Serial.print(F("run() starting new move out of bounds.\n  source: ("));
                Serial.print(source.x);
                Serial.print(F(", "));
                Serial.print(source.y);
                Serial.print(F(", "));
                Serial.print(source.z);
                Serial.print(F("), hSlope: "));
                Serial.print(hSlope[hHeading]);
                Serial.print(F(", vSlope: "));
                Serial.println(vSlope[vHeading]);
            }
            #endif
            #ifdef FP_DEBUG_RU
            Serial.print(F("run() new move. source: ("));
            Serial.print(source.x);
            Serial.print(F(", "));
            Serial.print(source.y);
            Serial.print(F(", "));
            Serial.print(source.z);
            Serial.print(F("), target: ("));
            Serial.print(target.x);
            Serial.print(F(", "));
            Serial.print(target.y);
            Serial.print(F(", "));
            Serial.print(target.z);
            Serial.print(") moveLength: ");
            Serial.println(moveLength);
            newMoveCount++;
            #endif

            // Start at t = 0 and with the dt needed to have batches consisting of batchSteps steps
            t = 0.0;
            dt = 1.0 / (moveLength / batchSteps);   // A batch every batchSteps
            nextCableSteps = p3DToCb(source);       //   starting at source
            nextPoint = source;

            #ifdef FP_DEBUG_ISR
            captured = false;
            printed = false;
            #endif

        }

        // Calculate the next batch of steps and mark them as available to the ISR. For the last batch use
        // the cable lengths at target since we could otherwise end at a slightly different place due to
        // errors from limited precision.
        float nextT = min(1.0, t + dt);                         // t for the next batch of steps
        fp_CableBundle startCableSteps = nextCableSteps;        // Cable lengths and 
        if (nextT < 0.999) {                                    // If it's not the last batch in the move
            nextPoint.x = source.x + nextT * (target.x - source.x);
            nextPoint.y = source.y + nextT * (target.y - source.y);
            nextPoint.z = source.z + nextT * (target.z - source.z);
            nextCableSteps = p3DToCb(nextPoint);
        } else {                                                // Else it's the last batch; use cable lengths at target
            nextCableSteps = p3DToCb(target);
        }
        t = nextT;
        #ifdef FP_DEBUG_GEO
        if (nextPoint.x < marginsMin.x || nextPoint.y < marginsMin.y || nextPoint.z < marginsMin.z ||
            nextPoint.x > marginsMax.x || nextPoint.y > marginsMax.y || nextPoint.z > marginsMax.z) {
            Serial.print(F("run() Heading out of bounds: ("));
            Serial.print(nextPoint.x);
            Serial.print(F(", "));
            Serial.print(nextPoint.y);
            Serial.print(F(", "));
            Serial.print(nextPoint.z);
            Serial.println(F(")"));
        }
        #endif
        float mostSteps = 0.0;
        for (byte i = 0; i < 4; i++) {
            nextPendingSteps[i] = nextCableSteps.c[i] - startCableSteps.c[i];
            if (nextPendingSteps[i] < 0) {
                nextPendingSteps[i] = -nextPendingSteps[i];
                nextShortening[i] = true;
            } else {
                nextShortening[i] = false;
            }
            if (nextPendingSteps[i] > mostSteps) {
                mostSteps = nextPendingSteps[i];
            }
        }
        long batchInterval = 1000000.0 * mostSteps / maxSpeed;  // How long (μs) it will take to do batch
        for (byte i = 0; i < 4; i++) {              // Set interval between steps
            nextDsInterval[i] = nextPendingSteps[i] == 0 ? batchInterval : batchInterval / nextPendingSteps[i];
        }
        #ifdef FP_DEBUG_RU
        Serial.print(F("run() New batch. t: "));
        Serial.print(t, 5);
        Serial.print(F(", pendingSteps: ("));
        for (byte i = 0; i < 4; i++) {
            Serial.print(nextShortening[i] ? -nextPendingSteps[i] : nextPendingSteps[i]);
            Serial.print(i == 3 ? F("), dsInterval: (") : F(", "));
        }
        for (byte i = 0; i < 4; i++) {
            Serial.print(nextDsInterval[i]);
            Serial.print(i == 3 ? F(").\n") : F(", "));
        }
        #endif
        #ifdef FP_DEBUG_RU
        if (t >= 1.0) {
            Serial.print(F("Last batch. moveCount: "));
            Serial.print(newMoveCount);
            Serial.print(F(", pendingSteps: ("));
            for (byte i = 0; i < 4; i++) {
                Serial.print(nextShortening[i] ? -nextPendingSteps[i] : nextPendingSteps[i]);
                Serial.print(i == 3 ? F("), dsInterval: (") : F(", "));
            }
            for (byte i = 0; i < 4; i++) {
                Serial.print(nextDsInterval[i]);
                Serial.print(i == 3 ? F(").\n") : F(", "));
            }
        }
        #endif
        nextReady = true;
        return true;
    }
    return false;
}

void FlyingPlatform::setMaxSpeed(float speed) {
    maxSpeed = speed;
}

fp_return_code FlyingPlatform::setSafetyMargins(
    long leftMargin, long rightMargin, 
    long frontMargin, long backMargin, 
    long bottomMargin, long topMargin) {
    if (leftMargin < 0 || rightMargin > space.x ||
        frontMargin < 0 || backMargin > space.y ||
        bottomMargin < 0 || topMargin > space.z) {
        return fp_oob;
    }
    marginsMin = fp_Point3D {leftMargin, frontMargin, bottomMargin};
    marginsMax = fp_Point3D {space.x - rightMargin, space.y - backMargin, space.z - topMargin};
    targetsMin = fp_Point3D {marginsMin.x + FP_TARGETS_BUFFER, marginsMin.y + FP_TARGETS_BUFFER, marginsMin.z + FP_TARGETS_BUFFER};
    targetsMax = fp_Point3D {marginsMax.x - FP_TARGETS_BUFFER, marginsMax.y - FP_TARGETS_BUFFER, marginsMax.z - FP_TARGETS_BUFFER};
    return fp_ok;
}

void FlyingPlatform::setBatchSize(long steps) {
    batchSteps = steps;
}

fp_return_code FlyingPlatform::setCurrentPosition (fp_Point3D tgt) {
    if (tgt.x < marginsMin.x || tgt.y < marginsMin.y || tgt.z < marginsMin.z ||
        tgt.x > marginsMax.x || tgt.y > marginsMax.y || tgt.z > marginsMax.z) {
            Serial.print(F("("));
            Serial.print(marginsMax.x);
            Serial.print(F(", "));
            Serial.print(marginsMax.y);
            Serial.print(F(", "));
            Serial.print(marginsMax.z);
            Serial.print(F(")\n"));
            return fp_oob;  // Can't do it: Target is out of bounds
        }
    if (isRunning()) {
        return fp_mov;      // Can't do it: A move is underway
    }
    fp_CableBundle cs = p3DToCb(tgt);
    ATOMIC_BLOCK(ATOMIC_FORCEON) {          // Techincally this shouldn't be needed since we're not running, but paranoia
        cableSteps[0] = cs.c[0];
        cableSteps[1] = cs.c[1];
        cableSteps[2] = cs.c[2];
        cableSteps[3] = cs.c[3];
    }
    source = target = tgt;
    hHeading = 0;
    vHeading = FP_LEVEL;

    isCalibrated = true;
    return fp_ok;;
}

fp_return_code FlyingPlatform::calibrate(fp_Point3D tgt) {
    if (!isEnabled) {
        return fp_dis;
    }
    fp_return_code rc = setCurrentPosition(tgt);
    if (rc != fp_ok) {
        return rc;
    }
    attachInterrupt(digitalPinToInterrupt(calClkPin), calClkISR, RISING);
    return fp_ok;
}

fp_return_code FlyingPlatform::enableOutputs() {
    digitalWrite(enablePin, LOW);   // Active LOW
    isEnabled = true;
    return fp_ok;
}

fp_return_code FlyingPlatform::disableOutputs() {
    if (isRunning()) {
        return fp_mov;  // Cant do it: A move is underway.
    }
    digitalWrite(enablePin, HIGH);  // Active LOW
    isEnabled = false;
    return fp_ok;
}

fp_return_code FlyingPlatform::moveTo(fp_Point3D tgt) {
    if (tgt.x < marginsMin.x || tgt.y < marginsMin.y || tgt.z < marginsMin.z ||
        tgt.x > marginsMax.x || tgt.y > marginsMax.y || tgt.z > marginsMax.z) {
        #ifdef FP_DEBUG_GEO
        Serial.print(F("moveTo() Out of bounds. tgt: ("));
        Serial.print(tgt.x);
        Serial.print(F(", "));
        Serial.print(tgt.y);
        Serial.print(F(", "));
        Serial.print(tgt.z);
        Serial.print(F("), Min: ("));
        Serial.print(marginsMin.x);
        Serial.print(F(", "));
        Serial.print(marginsMin.y);
        Serial.print(F(", "));
        Serial.print(marginsMin.z);
        Serial.print(F("), Max: ("));
        Serial.print(marginsMax.x);
        Serial.print(F(", "));
        Serial.print(marginsMax.y);
        Serial.print(F(", "));
        Serial.print(marginsMax.z);
        Serial.println(F(")."));
        #endif
        return fp_oob;      // Can't do it: Target is out of bounds
    }
    if (!isCalibrated) {
        return fp_ncp;          // Can't do it: No current position set
    }
    // Okay, go for it: Set things up to move to tgt
    target = tgt;
    newMove = true;                         // Indicate new move

    #ifdef FP_DEBUG_MT
    Serial.print(F("moveTo() from: ("));
    Serial.print(source.x);
    Serial.print(F(", "));
    Serial.print(source.y);
    Serial.print(F(", "));
    Serial.print(source.z);
    Serial.print(F("), to: ("));
    Serial.print(target.x);
    Serial.print(F(", "));
    Serial.print(target.y);
    Serial.print(F(", "));
    Serial.print(target.z);
    Serial.println(F(")."));
    #endif

    return fp_ok;
}

fp_return_code FlyingPlatform::moveBy(fp_Point3D delta) {
    if (!isCalibrated) {
        return fp_ncp;
    }
    fp_Point3D current = where();

    #ifdef FP_DEBUG_MB
    Serial.print(F("moveBy() from: ("));
    Serial.print(current.x);
    Serial.print(F(", "));
    Serial.print(current.y);
    Serial.print(F(", "));
    Serial.print(current.z);
    Serial.print(F("), by: ("));
    Serial.print(delta.x);
    Serial.print(F(", "));
    Serial.print(delta.y);
    Serial.print(F(", "));
    Serial.print(delta.z);
    Serial.println(F(")."));
    #endif

    return moveTo(fp_Point3D {current.x + delta.x, current.y + delta.y, current.z + delta.z});
}

fp_return_code FlyingPlatform::go() {
    if (!isCalibrated) {
        return fp_ncp;
    }
    return moveTo(newTarget());
}

fp_return_code FlyingPlatform::turn(fp_hTurns dir) {
    hChange = dir;
    if (isRunning() && !stopping) {
        return go();
    }
    return fp_ok;
}

fp_return_code FlyingPlatform::turn(fp_vTurns dir) {
    vChange = dir;
    if (isRunning() && !stopping) {
        return go();
    }
    return fp_ok;
}

void FlyingPlatform::stop() {
    stopping = true;
    detachInterrupt(digitalPinToInterrupt(calClkPin));  // Also stop calibration, if it's running
    isCalibrating = false;
    isCalibrated = false;

}

fp_Point3D FlyingPlatform::where() {
    if (!isCalibrated) {
        return fp_Point3D {0, 0, 0};
    }
    fp_CableBundle cs;
    ATOMIC_BLOCK(ATOMIC_FORCEON) { // Needed because the ISR might be actively updating cableSteps.
        for (byte i = 0; i < 4; i++) {
            cs.c[i] = cableSteps[i];
        }
    }
    fp_Point3D answer = cbToP3D(cs);

    #ifdef FP_DEBUG_GEO
    if (answer.x < marginsMin.x || answer.y < marginsMin.y || answer.z < marginsMin.z ||
        answer.x > marginsMax.x || answer.y > marginsMax.y || answer.z > marginsMax.z) {
        Serial.print(F("where() called while out of bounds. x: "));
        Serial.print(answer.x);
        Serial.print(F(", y: "));
        Serial.print(answer.y);
        Serial.print(F(", z: "));
        Serial.print(answer.z);
        Serial.print(F(", hSlope: "));
        Serial.print(hSlope[hHeading]);
        Serial.print(F(", cableSteps: ("));
        for (byte i = 0; i < 4; i++) {
            Serial.print(cs.c[i]);
            Serial.print(i < 3 ? F(", ") : F(")\n"));
        }
    }
    #endif

    return answer;
}

bool FlyingPlatform::isRunning() {
    return timerISRHasWork;
}

fp_Point3D FlyingPlatform::newTarget() {
    fp_Point3D here;
    if (timerISRHasWork) {
        here = cbToP3D(nextCableSteps); // If we're running, we'll start with next batch
    } else {
        here = where();                 // If we're stopped, we'll start where we are
    }
    #ifdef FP_DEBUG_GEO
    if (here.x < marginsMin.x || here.y < marginsMin.y || here.z < marginsMin.z ||
        here.x > marginsMax.x || here.y > marginsMax.y || here.z > marginsMax.z) {
        Serial.print(F("newTarget() starting out of bounds.\n  here: ("));
        Serial.print(here.x);
        Serial.print(F(", "));
        Serial.print(here.y);
        Serial.print(F(", "));
        Serial.print(here.z);
        Serial.print(F("), hSlope: "));
        Serial.print(hSlope[hHeading]);
        Serial.print(F(", vSlope: "));
        Serial.println(vSlope[vHeading]);
    }
    #endif
    // Figure y intercept
    float by = here.y - (hSlope[hHeading] * here.x);

    // Assume we should aim to hit the left or right of our allowed space (i.e., max or min x)
    float x = fp_xIsRising(hHeading) ? targetsMax.x : targetsMin.x;
    float y = hSlope[hHeading] * x + by;

    // If that puts x and y at (almost exactly) the same place as here, we're already up 
    // against the target boundary
    if (abs(here.x - x) < 1.0 && abs(here.y - y) < 1.0) {
        #ifdef FP_DEBUG_NT
        Serial.print(F("newTarget() At boundary from x. here: ("));
        Serial.print(here.x);
        Serial.print(F(", "));
        Serial.print(here.y);
        Serial.print(F(", "));
        Serial.print(here.z);
        Serial.println(F(")."));
        #endif
        return here;
    }

    // Calculate what z is based on vSlope x, here.x, y, here.y and here.z.
    float z = here.z + vSlope[vHeading] * sqrt((x - here.x) * (x - here.x) + (y - here.y) * (y - here.y));
    float x0 = x;
    float y0 = y;
    float z0 = z;

    // If aiming at the left or right would result in going beyond the front, 
    // back, top or bottom and we're not going parallel to the front/back, 
    // assume we should aim at the front or back. Which it is depends on whether 
    // x is rising and on hSlope. E.g., when x is rising and hSlope is positive, 
    // y is also rising so it's the back.
    if (y < marginsMin.y || y > marginsMax.y || z < marginsMin.z || z > marginsMax.z) {
        y = fp_xIsRising(hHeading) == (hSlope[hHeading] > 0) ? targetsMax.y : targetsMin.y;
        x = (y - by) / hSlope[hHeading];
    }

    // If that puts x and y at (almost exactly) the same place as here, we're already up 
    // against the target boundary
    if (abs(here.x - x) < 1.0 && abs(here.y - y) < 1.0) {
        #ifdef FP_DEBUG_NT
        Serial.print(F("newTarget() At boundary from y. here: ("));
        Serial.print(here.x);
        Serial.print(F(", "));
        Serial.print(here.y);
        Serial.print(F(", "));
        Serial.print(here.z);
        Serial.println(F(")."));
        #endif
        return here;
    }

    // Calculate what z is based on vSlope x, here.x, y, here.y and here.z.
    z = here.z + vSlope[vHeading] * sqrt((x - here.x) * (x - here.x) + (y - here.y) * (y - here.y));
    float x1 = x;
    float y1 = y;
    float z1 = z;

    // If all of the preceeding would result in going beyond the top or bottom, 
    // aim for the top or bottom.
    if (z < marginsMin.z || z > marginsMax.z) {
        z = z < targetsMin.z ? targetsMin.z : targetsMax.z;

        // The distance in the x-y plane from (here.x, here.y, here.z) to the point in the x-y 
        // plane where we would reach z, given vSlope is 
        //   d = (z - here.z) / vSlope.
        // The square of the same distance in terms of x, here.x, y and here.y is 
        //   d**2 = (x - here.x)**2 + (y - here.y)**2. 
        // We also know 
        //   (y - here.y) / (x - here.x) = hSlope.
        // With these and some algebra we can solve for x in terms of z: 
        //   x = here.x + (z - here.z) / (sqrt(hSlope**2 + 1) * (+-)vSlope)
        // the vSlope term is + if x is rising, - if it is falling
        if (fp_xIsRising(hHeading)){
            x = here.x + (z - here.z) / (sqrt(1 + hSlope[hHeading] * hSlope[hHeading]) * vSlope[vHeading]);
        } else {
            x = here.x - (z - here.z) / (sqrt(1 + hSlope[hHeading] * hSlope[hHeading]) * vSlope[vHeading]);
        }
        // Since we know hSlope, x and by, finding y is trivial
        y =  hSlope[hHeading] * x + by;
    }

    fp_Point3D answer = {(long)(x + 0.5), (long)(y+ 0.5), (long)(z + 0.5)};

    #ifdef FP_DEBUG_NT
    Serial.print(F("newTarget() result is "));
    if (answer.x < marginsMin.x || answer.y < marginsMin.y || answer.z < marginsMin.z ||
        answer.x > marginsMax.x || answer.y > marginsMax.y || answer.z > marginsMax.z) {
        Serial.print(F("out of bounds.\n  tgtType: "));
    } else {
        Serial.print(F("ok.\n tgeType: "));
    }
    #else
    // Assert: The answer should be inside of the margins
    if (answer.x < marginsMin.x || answer.y < marginsMin.y || answer.z < marginsMin.z ||
        answer.x > marginsMax.x || answer.y > marginsMax.y || answer.z > marginsMax.z) {
        Serial.print(F("Assertion failed in newTarget(): Target is outside the margins.\n  here: ("));
    #endif
        Serial.print(here.x);
        Serial.print(F(", "));
        Serial.print(here.y);
        Serial.print(F(", "));
        Serial.print(here.z);
        Serial.print(F("), target: ("));
        Serial.print(answer.x);
        Serial.print(F(", "));
        Serial.print(answer.y);
        Serial.print(F(", "));
        Serial.print(answer.z);
        Serial.print(F("), margins: (<"));
        Serial.print(marginsMin.x);
        Serial.print(F(">-<"));
        Serial.print(marginsMax.x);
        Serial.print(F(">, <"));
        Serial.print(marginsMin.y);
        Serial.print(F(">-<"));
        Serial.print(marginsMax.y);
        Serial.print(F(">, <"));
        Serial.print(marginsMin.z);
        Serial.print(F(">-<"));
        Serial.print(marginsMax.z);
        Serial.print(F(">)\n  x0: "));
        Serial.print(x0);
        Serial.print(F(", y0: "));
        Serial.print(y0);
        Serial.print(F(", z0: "));
        Serial.print(z0);
        Serial.print(F(", x1: "));
        Serial.print(x1);
        Serial.print(F(", y1: "));
        Serial.print(y1);
        Serial.print(F(", z1: "));
        Serial.print(z1);
        Serial.print(F(", x is "));
        Serial.print(fp_xIsRising(hHeading) ? F("rising") : F("falling"));
        Serial.print(F(", hSlope: "));
        Serial.print(hSlope[hHeading]);
        Serial.print(F(", vSlope: "));
        Serial.print(vSlope[vHeading]);
        Serial.print(F(", by: "));
        Serial.println(by);
    #ifndef FP_DEBUG_NT
        while (true) {
            // Spin
        }
    }
    #endif
    return answer;
}

fp_Point3D FlyingPlatform::cbToP3D(fp_CableBundle bundle) {
    fp_Point3D point;
    float dx = 0.0, dy = 0.0, r;
    // From Weisstein, Eric W. "Sphere-Sphere Intersection." From 
    // MathWorld--A Wolfram Web Resource. 
    // https://mathworld.wolfram.com/Sphere-SphereIntersection.html, the x
    // coordinate of the circle that is the intersection of two spheres, one
    // which is centered at the origin and has radius R and the other of which 
    // has radius r and is centered on the x axis at distance d along is given
    // by
    //      x = (d^2 - r^2 + R^2) / (2 * d) = d / 2 + (R^2 - r^2) / (2 * d)
    //
    // So we have
    point.x = 0.5 + space.x / 2.0 + (bundle.c[0] * bundle.c[0] - bundle.c[1] * bundle.c[1]) / (space.x * 2.0);
    //
    // Similarly for y
    point.y = 0.5 + space.y / 2.0 + (bundle.c[0] * bundle.c[0] - bundle.c[2] * bundle.c[2]) / (space.y * 2.0);

    // From the formula for a sphere and knowing x and y we can use any of the 
    // cableSteps[0 .. 3] to calculate z. we'll use the largest for the best 
    // resolution.
    r = bundle.c[0];
    for (int8_t n = 1; n < 4; n++) {
        if (bundle.c[n] > r) {
            r = bundle.c[n];
            dx = n % 2 ? space.x : 0.0;
            dy = n > 1 ? space.y : 0.0;            
        }
    }
    point.z = 0.5 + space.z - sqrt(r * r - (point.x - dx) * (point.x - dx) - (point.y - dy) * (point.y - dy));
    return point;
}

fp_CableBundle FlyingPlatform::p3DToCb(fp_Point3D point) {
    fp_CableBundle bundle;
    bundle.c[0] = 0.5 + sqrt(point.x * point.x + point.y * point.y + (space.z - point.z) * (space.z - point.z));
    bundle.c[1] = 0.5 + sqrt((space.x - point.x) * (space.x - point.x) + point.y * point.y + (space.z - point.z) * (space.z - point.z));
    bundle.c[2] = 0.5 + sqrt(point.x * point.x + (space.y - point.y) * (space.y - point.y) + (space.z - point.z) * (space.z - point.z));
    bundle.c[3] = 0.5 + sqrt((space.x - point.x) * (space.x - point.x) + (space.y - point.y) * (space.y - point.y) + (space.z - point.z) * (space.z - point.z));
    return bundle;
}