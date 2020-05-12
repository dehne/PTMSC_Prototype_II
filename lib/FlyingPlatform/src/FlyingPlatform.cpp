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
 * FlyingPlatform V0.3, May 2020
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
 * Variables held in common with the ISR. I don't like this, but I can find no 
 * clean way of sharing variables with an ISR.
 * 
 **/
volatile static bool nextReady;             // Lock for sharing nextXxxx variables. true => ISR owns false => run()
static bool nextShortening[4];              // Buffer for Whether the cable is shortening or lengthening in next batch
static long nextPendingSteps[4];            // Buffer for pendingSteps in next batch of steps
static unsigned long nextDsInterval[4];     // Buffer for interval between steps (μs) in next batch

volatile static bool workingOnBatch;        // True if ISR still has steps to take. (Owned by ISR)
volatile static long cableSteps[4];         // The current length of the cables, in steps (Owned by ISR; use ATOMIC access)
static byte dirPin[4];                      // Direction digital output pins. High means lengthen cables (Owned by object)
static byte stepPin[4];                     // Step digital output pins. Normally LOW. Pulse HIGH to step. (Owned by object)

#ifdef FP_DEBUG_ISR
volatile static bool captured = false;
volatile static bool printed = false;
volatile static unsigned long tcurMicros;
volatile static unsigned long tTargetMicros0;
volatile static unsigned long tDsInterval0;
#endif


/**
 * 
 * Interrupt service routine. Invoked whenever the count in Timer 0 reaches 
 * the value specified in register OCR0A. Because Timer 0 runs throught its
 * 256 values once every 1/976.5625 of a second, that's how often the 
 * interrupt would occur if we didn't mess with OCR0A. But, to optimize 
 * dispatching steps to the motors, we do.
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
 * Finally, if there is a step whose time will come due inside the next ms, we 
 * adjust the value of OCR0A so that an interrupt will ossur at the correct 
 * time. If there's isn't one, we leave OCR0A alone which causes the next
 * interrupt to occur in 1 ms.
 * 
 **/
ISR(TIMER0_COMPA_vect) {
    static byte nFinished = 4;                              // Number of motors finished with this batch
    static long pendingSteps[4];                            // Number of remaining steps by which to change each cable
    static bool shortening[4];                              // True if shortening cable
    static unsigned long dsInterval[4];                     // Interval between steps (μs)
    static unsigned long targetMicros[4] =  {0, 0, 0, 0};   // When to dispatch next step

    if (nFinished == 4 && !nextReady) {                     // No work if finished with batch but no new batch ready
        workingOnBatch = false;
        return;
    }
    unsigned long curMicros = micros();
    if (nFinished == 4) {                                   // Finished with batch and new batch ready
        // Copy batch data from nextXxx variables to ISR's variables; set rotation directions
        for (byte i = 0; i < 4; i++) {
            pendingSteps[i] = nextPendingSteps[i];
            shortening[i] = nextShortening[i];
            if (shortening[i]) {
                digitalWrite(dirPin[i], LOW);   // Set to shorten cable
            } else {
                digitalWrite(dirPin[i], HIGH);  // Set to lengthen cable
            }
            dsInterval[i] = nextDsInterval[i];
            targetMicros[i] = curMicros;
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
        }
        if (targetMicros[i] < shortestT) {
        shortestT = targetMicros[i];
        }
    }

    workingOnBatch = nFinished != 4; // Indicate whether we still have work
    #ifdef FP_DEBUG_LED
    digitalWrite(A5, workingOnBatch ? HIGH : LOW);
    #endif


    // shortestT now holds the micros() at which the next step(s) should be 
    // taken. Figure out what to set OCR0A to to interrupt appropriately. No 
    // need to mess with OCR0A if shortestT is 1 ms or more.
    if (shortestT - curMicros < 1000) {
        int timerCountsToGo = (shortestT - curMicros) / 4;
        byte newOCR0A = (timerCountsToGo + OCR0A) & 0xFF;
        OCR0A = newOCR0A;
    }
}

FlyingPlatform::FlyingPlatform(
    byte pin0D, byte pin0P, 
    byte pin1D, byte pin1P, 
    byte pin2D, byte pin2P, 
    byte pin3D, byte pin3P,
    byte pinEn,
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
        space = fp_Point3D {spaceWidth, spaceDepth, spaceHeight};
        marginsMin = fp_Point3D {0, 0, 0};
        marginsMax = fp_Point3D {spaceWidth, spaceDepth, spaceHeight};
    }

void FlyingPlatform::begin() {
    pinMode(enablePin, OUTPUT);
    for (byte i = 0; i < 4; i++) {
        pinMode(dirPin[i], OUTPUT);
        pinMode(stepPin[i], OUTPUT);
        cableSteps[i] = 0;          // ISR not yet running; no ATOMIC stuff needed
        nextPendingSteps[i] = 0;
        nextDsInterval[i] = 0;
        nextShortening[i] = false;
    }
    currentIsSet = false;
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

    #ifdef FP_DEBUG_LED
    pinMode(A5, OUTPUT);
    digitalWrite(A5, LOW);
    #endif

    OCR0A = FP_STARTING_COUNT;      // Value Timer0 is compared to to cause interrupt.
    TIMSK0 |= _BV(OCIE0A);          // Turn on OCR0A compare interrupts
}

bool FlyingPlatform::run() {
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
    
    if (!currentIsSet || !isEnabled) {
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
        // If there's a new heading and we're not stopped or trying to stop
        if (changed && t != 1.0 && !stopping) {
            moveTo(newTarget());
        }
        turnMicros = curMicros;
    }

    // If all done with move and nothing new has been scheduled, no use doing more work.
    if (t == 1.0 && !newMove) {
        stopping = false;
        return false;
    }

    // If it's time to calculate the next batch of steps
    if (!nextReady) {
        // If we're to go off in a new direction, set up for first batch of the move from wherever we are to target
        if (newMove) {
            source = where();
            // Cable lengths at source
            nextCableSteps[0] = sqrt(source.x * source.x + source.y * source.y + (space.z - source.z) * (space.z - source.z));
            nextCableSteps[1] = sqrt((space.x - source.x) * (space.x - source.x) + source.y * source.y + (space.z - source.z) * (space.z - source.z));
            nextCableSteps[2] = sqrt(source.x * source.x + (space.y - source.y) * (space.y - source.y) + (space.z - source.z) * (space.z - source.z));
            nextCableSteps[3] = sqrt((space.x - source.x) * (space.x - source.x) + (space.y - source.y) * (space.y - source.y) + (space.z - source.z) * (space.z - source.z));

            // Start at t = 0 and with the dt needed to have batches consisting of batchSteps steps
            t = 0.0;
            float moveLength = sqrt(
                (target.x - source.x) * (target.x - source.x) + 
                (target.y - source.y) * (target.y - source.y) + 
                (target.z - source.z) * (target.z - source.z));
            dt = 1.0 / (moveLength / batchSteps);   // A batch every batchSteps
            newMove = false;

            #ifdef FP_DEBUG_ISR
            captured = false;
            printed = false;
            #endif
        }

        float nextT;                                        // t for the next batch of steps
        float xsts, sxxsts;                                 // x(nextT)**2 and (space.x - x(nextT))**2
        float ysts, syysts;                                 // y(nextT)**2 and (space.y - y(nextT))**2
        float szzsts;                                       // (space.z - z(nextT))**2
        float startCableSteps[4];                           // Cable lengths at start of the new batch

        // Calculate the next batch of steps and mark them as available to the ISR
        nextT = min(1.0, t + dt);                           // t for the next batch
        xsts = (1 - nextT) * source.x + nextT * target.x;   // x(nextT)
        sxxsts = space.x - xsts;                            // space.x - x(nextT)
        ysts = (1 - nextT) * source.y + nextT * target.y;   // y(nextT)
        syysts = space.y - ysts;                            // space.y - y(nextT)
        xsts *= xsts;                                       // x(nextT)**2
        sxxsts *= sxxsts;                                   // (space.x - x(nextT))**2
        ysts *= ysts;                                       // y(nextT)**2
        szzsts = space.z -                                  // space.z - z(nextT)
            ((1 - nextT) * source.z + nextT * target.z);    //   (z(nextT) not needed independently)
        syysts *= syysts;                                   // (space.y - y(nextT))**2
        szzsts *= szzsts;                                   // (space.z - z(nextT))**2
        startCableSteps[0] = nextCableSteps[0];             // Batch starts with these lengths
        startCableSteps[1] = nextCableSteps[1];
        startCableSteps[2] = nextCableSteps[2];
        startCableSteps[3] = nextCableSteps[3];
        nextCableSteps[0] = sqrt(xsts + ysts + szzsts);     //   and ends with these
        nextCableSteps[1] = sqrt(sxxsts + ysts + szzsts);
        nextCableSteps[2] = sqrt(xsts + syysts + szzsts);
        nextCableSteps[3] = sqrt(sxxsts + syysts + szzsts);
        t = nextT;
        float mostSteps = 0.0;
        for (byte i = 0; i < 4; i++) {
            nextPendingSteps[i] = nextCableSteps[i] - startCableSteps[i];
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
            nextDsInterval[i] = nextPendingSteps[i] == 0 ? FP_MAX_JITTER : batchInterval / nextPendingSteps[i];
        }
        #ifdef FP_DEBUG_RU
        Serial.print(F("run() New batch. pendingSteps: ("));
        for (byte i = 0; i < 4; i++) {
            Serial.print(nextShortening[i] ? -nextPendingSteps[i] : nextPendingSteps[i]);
            Serial.print(i == 3 ? F("), dsInterval: (") : F(", "));
        }
        for (byte i = 0; i < 4; i++) {
            Serial.print(nextDsInterval[i]);
            Serial.print(i == 3 ? F(").\n") : F(", "));
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
    marginsMax = fp_Point3D {rightMargin, backMargin, topMargin};
    return fp_ok;
}

void FlyingPlatform::setBatchSize(long steps) {
    batchSteps = steps;
}

fp_return_code FlyingPlatform::setCurrentPosition (fp_Point3D tgt) {
    if (tgt.x < marginsMin.x || tgt.y < marginsMin.y || tgt.z < marginsMin.z ||
        tgt.x > marginsMax.x || tgt.y > marginsMax.y || tgt.z > marginsMax.z) {
            return fp_oob;  // Can't do it: Target is out of bounds
        }
    if (isRunning()) {
        return fp_mov;      // Can't do it: A move is underway
    }
    float cs0 = sqrt(tgt.x * tgt.x + tgt.y * tgt.y + (space.z - tgt.z) * (space.z - tgt.z));
    float cs1 = sqrt((space.x - tgt.x) * (space.x - tgt.x) + tgt.y * tgt.y + (space.z - tgt.z) * (space.z - tgt.z));
    float cs2 = sqrt(tgt.x * tgt.x + (space.y - tgt.y) * (space.y - tgt.y) + (space.z - tgt.z) * (space.z - tgt.z));
    float cs3 = sqrt((space.x - tgt.x) * (space.x - tgt.x) + (space.y - tgt.y) * (space.y - tgt.y) + (space.z - tgt.z) * (space.z - tgt.z));
    ATOMIC_BLOCK(ATOMIC_FORCEON) {          // Techincally this shouldn't be needed since we're not running, but it won't hurt
        cableSteps[0] = cs0;
        cableSteps[1] = cs1;
        cableSteps[2] = cs2;
        cableSteps[3] = cs3;
    }
    source = target = tgt;
    hHeading = 0;
    vHeading = FP_LEVEL;

    currentIsSet = true;
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
        #ifdef FP_DEBUG_MT
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
    if (!currentIsSet) {
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
    if (!currentIsSet) {
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
    if (!currentIsSet) {
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
    t = 1.0;                    // Stop once the current batch is complete.
    stopping = true;
}

fp_Point3D FlyingPlatform::where() {
    if (!currentIsSet) {
        return fp_Point3D {0, 0, 0};
    }
    float dx = 0.0, dy = 0.0, r, x, y, z;
    ATOMIC_BLOCK(ATOMIC_FORCEON) { // ATOMIC needed because we can be called while the ISR is actively updating cableSteps.
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
    x = space.x / 2.0 + (cableSteps[0] * cableSteps[0] - cableSteps[1] * cableSteps[1]) / (space.x * 2.0);
    //
    // Similarly for y
    y = space.y / 2.0 + (cableSteps[0] * cableSteps[0] - cableSteps[2] * cableSteps[2]) / (space.y * 2.0);

    // From the formula for a sphere and knowing x and y we can use any of the 
    // cableSteps[0 .. 3] to calculate z. we'll use the largest for the best 
    // resolution.
        r = cableSteps[0];
        for (int8_t n = 1; n < 4; n++) {
            if (cableSteps[n] > r) {
                r = cableSteps[n];
                dx = n % 2 ? space.x : dx;
                dy = n > 1 ? space.y : dy;            
            }
        }
    }
    z = space.z - sqrt(r * r - (x - dx) * (x - dx) - (y - dy) * (y - dy));

    #ifdef FP_DEBUG_W
    Serial.print(F("where() r: "));
    Serial.print(r);
    Serial.print(F(", dx: "));
    Serial.print(dx);
    Serial.print(F(", dy: "));
    Serial.print(dy);
    Serial.print(F(", x: "));
    Serial.print(x);
    Serial.print(F(", y: "));
    Serial.print(y);
    Serial.print(F(", z: "));
    Serial.println(z);
    #endif

    return fp_Point3D {(long)x, (long)y, (long)z};
}

bool FlyingPlatform::isRunning() {
    return t != 1.0 || workingOnBatch;
}

fp_Point3D FlyingPlatform::newTarget() {
    fp_Point3D here = where();
    // Figure y intercept
    float by = here.y - (hSlope[hHeading] * here.x);

    // Assume we'll aim to hit the left or right wall (i.e., along the x axis)
    float x = fp_xIsRising(hHeading) ? marginsMax.x : marginsMin.x;
    float y = hSlope[hHeading] * x + by;

    // If that would result in hitting the front or back, assume that's what we hit
    if (y < marginsMin.y || y > marginsMax.y) {
        y = y < marginsMin.y ? marginsMin.y : marginsMax.y;
        x = (y - by) / hSlope[hHeading];
    }

    // Calculate what z would be based on x and y. It's z = vSlope[vHeading] * (+-sqrt(x**2 + y**2)) + bz
    // Which it is depends on whether the horoizontal motion is getting closer to or farther from the origin
    float bz;
    if ((here.x * here.x + here.y * here.y) - (x * x + y * y) <= 0) {   // If horizontal motion is heading away from origin
            bz = here.z - vSlope[vHeading] * sqrt(here.x * here.x + here.y * here.y);
        } else {                                                        // Otherwise horizontal motion is heading towards origin
            bz = here.z + vSlope[vHeading] * sqrt(here.x * here.x + here.y * here.y);
        }
    float z = vSlope[vHeading] * sqrt(x * x + y * y) + bz;

    // Assert: Should not happen
    if ((vHeading < FP_LEVEL && z > here.z) || (vHeading > FP_LEVEL && z < here.z)) {
        Serial.print(F("Assertion failed in newTarget(): Wrong vertical direction. here: ("));
        Serial.print(here.x);
        Serial.print(F(", "));
        Serial.print(here.y);
        Serial.print(F(", "));
        Serial.print(here.z);
        Serial.print(F("), target: ("));
        Serial.print(x);
        Serial.print(F(", "));
        Serial.print(y);
        Serial.print(F(", "));
        Serial.print(z);
        Serial.print(F("), hSlope: "));
        Serial.print(hSlope[hHeading]);
        Serial.print(F(", vSlope: "));
        Serial.print(vSlope[vHeading]);
        Serial.print(F(", by: "));
        Serial.print(by);
        Serial.print(F(", bz: "));
        Serial.println(bz);
    }
    //If that would result in hitting the top or bottom, go for that, instead.
    if (z < marginsMin.z || z > marginsMax.z) {
        z = z < marginsMin.z ? marginsMin.z : marginsMax.z;

        #ifdef FP_DEBUG_NT
        Serial.print(F("newTarget() off bottom or top. here: ("));
        Serial.print(here.x);
        Serial.print(F(", "));
        Serial.print(here.y);
        Serial.print(F(", "));
        Serial.print(here.z);
        Serial.print(F("), hSlope: "));
        Serial.print(hSlope[hHeading],6);
        Serial.print(F(", vSlope: "));
        Serial.print(vSlope[vHeading],6);
        Serial.print(F("\n  y-based target: ("));
        Serial.print(x);
        Serial.print(F(", "));
        Serial.print(y);
        Serial.print(F(", "));
        Serial.print(z);
        Serial.println(F(")."));
        #endif


        // The distance in the horizontal plane from (here.x, here.y) to the point in x-y where we would
        // go outside the flying area given z, here.z and vSlope is d = (z - here.z) / vSlope.
        // The square of the same distance in terms of x, here.x, y and here.y is 
        // d**2 = (x - here.x)**2 + (y - here.y)**2. We also know (y - here.y) / (x - here.x) = hSlope.
        // With these and some algebra we can solve for x in terms of z: 
        // (z - here.z) / (sqrt(hSlope**2 + 1) * vSlope) + here.x
        x = (z - here.z) / (sqrt(hSlope[hHeading] * hSlope[hHeading] + 1) * vSlope[vHeading]) + here.x;
        y =  hSlope[hHeading] * (x - here.x) + here.y;

        #ifdef FP_DEBUG_NT
        Serial.print(F("  z-based target: ("));
        Serial.print(x);
        Serial.print(F(", "));
        Serial.print(y);
        Serial.print(F(", "));
        Serial.print(z);
        Serial.println(F(")."));
        #endif
    }
    return fp_Point3D {(long)x, (long)y, (long)z};
}
