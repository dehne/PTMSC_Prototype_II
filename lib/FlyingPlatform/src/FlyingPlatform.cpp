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
 * FlyingPlatform V0.1, April 2020
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
        currentIsSet = false;
        digitalWrite(enablePin, LOW);   // Enable (active LOW) motor drivers
        isEnabled = true;

        batchSteps = FP_BATCH_STEPS;
        t = 1.0;
        for (byte i = 0; i < 4; i++) {
            cableSteps[i] = pendingSteps[i] = 0;
            shortening[i] = false;
        }
        newMove = false;
        stopping = false;
        nextBatchPhase = 0;
        maxSpeed = FP_MAX_SPEED;
        hHeading = 0;
        hChange = fp_straight;
        vHeading = FP_LEVEL;
        vChange = fp_level;
        turnMicros = 0;
    }

void FlyingPlatform::begin() {
    pinMode(enablePin, OUTPUT);
    for (byte i = 0; i < 4; i++) {
        pinMode(dirPin[i], OUTPUT);
        pinMode(stepPin[i], OUTPUT);
    }
    #ifdef FP_DEBUG_LED
    pinMode(A5, OUTPUT);
    digitalWrite(A5, LOW);
    #endif
}

bool FlyingPlatform::run() {
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
        if (changed && t != 1.0 && !stopping) {     // If there's a new heading and we're not trying to stop
            moveTo(newTarget());
        }
        turnMicros = curMicros;
    }

    // Take any pending steps for which the time is ripe
    byte nFinished = 0;
    for (byte i = 0; i < 4; i++) {
        if (curMicros - dsMicros[i] >= dsInterval[i] && pendingSteps[i] > 0) {
            digitalWrite(stepPin[i], HIGH);    // Move stepper[i] one step
            digitalWrite(stepPin[i], LOW);
            pendingSteps[i]--;
            cableSteps[i] += shortening[i] ? -1 : 1;
            dsMicros[i] = curMicros;
        }
        if (pendingSteps[i] == 0) {
            nFinished++;
        }
    }

    // If all done with move and nothing new has been scheduled, no use doing more work.
    if (t == 1.0 && nFinished == 4 && !newMove) {
        #ifdef FP_DEBUG_LED
        digitalWrite(A5, LOW);
        #endif
        stopping = false;
        return false;
    }

    // If we're to go off in a new direction
    if (newMove) {
        t = 0.0;
        nextBatchPhase = 0;
        newMove = false;
    }

    // If there's a next batch of steps, calculate what they are over 5 passes through
    // run() to spread the calculation involved out over time. Then, when the current
    // batch of steps has been carried out, dispatch the next batch for processing in
    // the sixth pass.
    switch (nextBatchPhase) {
        case 0:
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
            nextBatchPhase++;
            break;
        case 1:
            nextCableSteps[0] = sqrt(xsts + ysts + szzsts);
            nextBatchPhase++;
            break;
        case 2:
            nextCableSteps[1] = sqrt(sxxsts + ysts + szzsts);
            nextBatchPhase++;
            break;
        case 3:
            nextCableSteps[2] = sqrt(xsts + syysts + szzsts);
            nextBatchPhase++;
            break;
        case 4:
            nextCableSteps[3] = sqrt(sxxsts + syysts + szzsts);
            nextBatchPhase++;
            break;
        case 5:
            if (nFinished == 4) {
                t = nextT;
                float longest = 0.0;
                for (byte i = 0; i < 4; i++) {
                    pendingSteps[i] = nextCableSteps[i] - cableSteps[i];
                    if (pendingSteps[i] < 0) {
                        pendingSteps[i] = -pendingSteps[i];
                        digitalWrite(dirPin[i], LOW);       // Set direction anticlockwise
                        shortening[i] = true;               //   to shorten the cable
                    } else {
                        digitalWrite(dirPin[i], HIGH);      // Set direction clockwise
                        shortening[i] = false;              //   to lengthen the cable
                    }
                    if (pendingSteps[i] > longest) {
                        longest = pendingSteps[i];
                    }
                }
                for (byte i = 0; i < 4; i++) {              // Set interval between steps
                    dsInterval[i] = (1000000.0 / maxSpeed) * (longest / max(pendingSteps[i], 1));
                }
            }
            nextBatchPhase = 0;
            #ifdef FP_DEBUG_LL
            Serial.print(F("run() New batch. pendingSteps: ("));
            for (byte i = 0; i < 4; i++) {
                Serial.print(shortening[i] ? -pendingSteps[i] : pendingSteps[i]);
                Serial.print(i == 3 ? F(").\n") : F(", "));
            }
            #endif
            break;
    }
    #ifdef FP_DEBUG_LED
    digitalWrite(A5, HIGH);
    #endif
    return true;
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
    cableSteps[0] = sqrt(tgt.x * tgt.x + tgt.y * tgt.y + (space.z - tgt.z) * (space.z - tgt.z));
    cableSteps[1] = sqrt((space.x - tgt.x) * (space.x - tgt.x) + tgt.y * tgt.y + (space.z - tgt.z) * (space.z - tgt.z));
    cableSteps[2] = sqrt(tgt.x * tgt.x + (space.y - tgt.y) * (space.y - tgt.y) + (space.z - tgt.z) * (space.z - tgt.z));
    cableSteps[3] = sqrt((space.x - tgt.x) * (space.x - tgt.x) + (space.y - tgt.y) * (space.y - tgt.y) + (space.z - tgt.z) * (space.z - tgt.z));
    target = tgt;
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
            return fp_oob;      // Can't do it: Target is out of bounds
        }
    if (!currentIsSet) {
        return fp_ncp;          // Can't do it: No current position set
    }
    // Okay, go for it: Set things up to move from where we are to tgt
    target = tgt;
    source = where();
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
    my = (target.y - source.y) / (target.x - source.x);
    mz = (target.z - source.z) / sqrt((target.x - source.x) * (target.x - source.x) + (target.y - source.y) * (target.y - source.y));
    by = source.y - my * source.x;
    bz = source.z - mz * sqrt(source.x * source.x + source.y * source.y);
    moveLength = sqrt(
        (target.x - source.x) * (target.x - source.x) + 
        (target.y - source.y) * (target.y - source.y) + 
        (target.z - source.z) * (target.z - source.z));
    dt = 1.0 / (moveLength / batchSteps);   // A batch every batchSteps
    newMove = true;                         // Indicate new move
    return fp_ok;
}

fp_return_code FlyingPlatform::moveBy(fp_Point3D delta) {
    if (!currentIsSet) {
        return fp_ncp;
    }
    fp_Point3D current = where();
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
    float x = space.x / 2.0 + (cableSteps[0] * cableSteps[0] - cableSteps[1] * cableSteps[1]) / (space.x * 2.0);
    //
    // Similarly for y
    float y = space.y / 2.0 + (cableSteps[0] * cableSteps[0] - cableSteps[2] * cableSteps[2]) / (space.y * 2.0);

    // From the formula for a sphere and knowing x and y we can use any of the 
    // cableSteps[0 .. 3] to calculate z. we'll use the largest for the best 
    // resolution.
    float r = cableSteps[0];
    float dx = 0.0;
    float dy = 0.0;
    for (int8_t n = 1; n < 4; n++) {
        if (cableSteps[n] > r) {
            r = cableSteps[n];
            dx = n % 2 ? space.x : dx;
            dy = n > 1 ? space.y : dy;            
        }
    }
    float z = space.z - sqrt(r * r - (x - dx) * (x - dx) - (y - dy) * (y - dy));
    return fp_Point3D {(long)x, (long)y, (long)z};
}

bool FlyingPlatform::isRunning() {
    return t != 1.0 || pendingSteps[0] != 0 || pendingSteps[1] != 0 || pendingSteps[2] != 0 || pendingSteps[3] != 0;
}

fp_Point3D FlyingPlatform::newTarget() {
    fp_Point3D here = where();
    // Figure y and z intercepts
    float by = here.y - (hSlope[hHeading] * here.x);
    float bz = here.z - vSlope[vHeading] * sqrt(here.x * here.x + here.y * here.y);

    // Assume we'll aim to hit the left or right wall (i.e., along the x axis)
    float x = fp_xIsRising(hHeading) ? marginsMax.x : marginsMin.x;
    float y = hSlope[hHeading] * x + by;
    float z = vSlope[vHeading] * sqrt(x * x + y * y) + bz;

    // If that would result in hitting the front or back, assume that's what we hit
    if (y < marginsMin.y || y > marginsMax.y) {
        y = y < marginsMin.y ? marginsMin.y : marginsMax.y;
        x = (y - by) / hSlope[hHeading];
        z = vSlope[vHeading] * sqrt(x * x + y * y) + bz;
    }

    //If that would result in hitting the top or bottom, go for that, instead.
    if (z < marginsMin.z || z > marginsMax.z) {
        z = z < marginsMin.z ? marginsMin.z : marginsMax.z;
        // The formula for computing x from z, my, mz, by and bz is pretty horrible:
        // x = -my * by + sqrt(4 * my**2 * by**2 - 4 * (my**2 + 1) * (by**2 - (z - bz)**2 / mz**2)) / (my**2 + 1)
        // It's the solution in x to y = my * x + by and z = mz * sqrt(x**2 + y**2) + bz using the
        // quadratic equation.
        float discriminant = 4 * hSlope[hHeading] * hSlope[hHeading] * (by * by) - 
            4 * (hSlope[hHeading] * hSlope[hHeading] + 1) * 
            ((by * by) - ((z - bz) * (z - bz)) / (vSlope[vHeading] * vSlope[vHeading]));
        if (discriminant < 0.0) {
            Serial.print(F("Discriminant in top-bottom target calculation is "));
            Serial.print(discriminant);
            Serial.print(F(". here: ("));
            Serial.print(here.x);
            Serial.print(F(", "));
            Serial.print(here.y);
            Serial.print(F(", "));
            Serial.print(here.z);
            Serial.print(F("), hSlope: "));
            Serial.print(hSlope[hHeading]);
            Serial.print(F(", vSlope: "));
            Serial.print(vSlope[vHeading]);
            Serial.print(F(", by: "));
            Serial.print(by);
            Serial.print(F(", bz: "));
            Serial.print(bz);
            Serial.println(F(".\nGood luck with that."));
            while (1) {
                // Halt: Don't risk crashing mechanism.
            }
        }
        x = -hSlope[hHeading] * by + sqrt(discriminant) / 
            (hSlope[hHeading] * hSlope[hHeading] + 1);
        y =  hSlope[hHeading] * x + by;
    }
    #ifdef FP_DEBUG_NT
    Serial.print(F("newTarget() here: ("));
    Serial.print(here.x);
    Serial.print(F(", "));
    Serial.print(here.y);
    Serial.print(F(", "));
    Serial.print(here.z);
    Serial.print(F("), hHeading: "));
    Serial.print(hHeading);
    Serial.print(F(", vHeading: "));
    Serial.print(vHeading);
    Serial.print(F(", new target: ("));
    Serial.print(x);
    Serial.print(F(", "));
    Serial.print(y);
    Serial.print(F(", "));
    Serial.print(z);
    Serial.println(F(")."));
    #endif
    return fp_Point3D {(long)x, (long)y, (long)z};
}
