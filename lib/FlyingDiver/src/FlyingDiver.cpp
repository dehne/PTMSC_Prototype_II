/****
 * 
 * This file is a portion of the package FlyingDiver, a library that provides 
 * an Arduino sketch with the ability to control the diver for the Port 
 * Townsend Marine Science Center's pinto abalone exhibit.
 * 
 * See FlyingDiver.h for details
 * 
 *****
 * 
 * FlyingDiver V0.2, March 2020
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

#include "FlyingDiver.h"

/**
 * 
 * Returns the length of the cable needed to position diver at the given point 
 * in space. See the PTMSC project paper for the geometry 
 * explanation. (It's just 3D Pythagoras.)
 * 
 * Parameters
 *      n   The cable for which the length is required
 *      p   The given point
 * 
 **/
int32_t cableLength(int8_t cable, Point3D p) {
    #ifdef FD_DEBUG_CL
        Serial.print(F("cablelength["));
        Serial.print(cable);
        Serial.print(F("] ("));
        Serial.print(p.x);
        Serial.print(F(", "));
        Serial.print(p.y);
        Serial.print(F(", "));
        Serial.print(p.z);
        Serial.print(F("): "));
    #endif
    int32_t l;
    switch (cable) {
        case 0: // front-left cable
            l = sqrt(p.x * p.x + p.y * p.y + (FD_Z_SIZE - p.z) * (FD_Z_SIZE - p.z))  + 0.5;
            break;
        case 1: // front-right cable
            l = sqrt((FD_X_SIZE - p.x) * (FD_X_SIZE - p.x) + p.y * p.y + (FD_Z_SIZE - p.z) * (FD_Z_SIZE - p.z)) + 0.5;
            break;
        case 2: // rear-left cable
            l = sqrt(p.x * p.x + (FD_Y_SIZE - p.y) * (FD_Y_SIZE - p.y) + (FD_Z_SIZE - p.z) * (FD_Z_SIZE - p.z)) + 0.5;
            break;
        case 3: // rear-right cable
            l = sqrt((FD_X_SIZE - p.x) * (FD_X_SIZE - p.x) + (FD_Y_SIZE - p.y) * (FD_Y_SIZE - p.y) + (FD_Z_SIZE - p.z) * (FD_Z_SIZE - p.z)) + 0.5;
            break;
        default:// should not happen
            return 0;
    }
    #ifdef FD_DEBUG_CL
        Serial.println(l);
    #endif
    return l;
}

/**
 * 
 * Returns true if the specified point is inside the volume the diver is
 * allowed to visit; false otherwise.
 * 
 * Parameter
 *  p   The specified point
 * 
 **/
bool isInBounds(Point3D p){
    if (p.x > FD_X_SIZE - FD_X_SAG || p.y > FD_Y_SIZE - FD_Y_SAG || p.z > FD_Z_SIZE - FD_Z_SAG ||
        p.x < FD_X_SAG || p.y < FD_Y_SAG || p.z < FD_Z_SAG) {
            return false;
        }
        return true;
}
FlyingDiver::FlyingDiver(int8_t pin0D, int8_t pin0P, int8_t pin1D, int8_t pin1P,
                         int8_t pin2D, int8_t pin2P, int8_t pin3D, int8_t pin3P,
                         int8_t pinEn) {
    // Make the four stepper objects, attaching them to the requisite pins
    stepper[0] = AccelStepper(AccelStepper::DRIVER, pin0P, pin0D);
    stepper[1] = AccelStepper(AccelStepper::DRIVER, pin1P, pin1D);
    stepper[2] = AccelStepper(AccelStepper::DRIVER, pin2P, pin2D);
    stepper[3] = AccelStepper(AccelStepper::DRIVER, pin3P, pin3D);

    // Set up the Multistepper so we can add the steppers to it as they are created
    multiStepper = MultiStepper();
    
    // Set motor driver parameters and enable them
    for (int8_t n = 0; n < 4; n++) {
        stepper[n].setMaxSpeed(FD_MAX_SPEED);
        stepper[n].setPinsInverted(false, false, true);       // Enable on TB6600 is inverted
        stepper[n].setEnablePin(pinEn);
        #if defined(FD_DEBUG_MT) || defined(FD_DEBUG_GO)
            started[n] = false;
        #endif
        multiStepper.addStepper(stepper[n]);
    }

}

void FlyingDiver::run() {
    if (!isCalibrated) {
        return;
    }
    unsigned long now = millis();
    bool updateCourse = false;
    if (isRunning() && now - turnClock > FD_TURN_MS){
        if (turning != straight) {
            heading = turning == left ? (heading + 1) % FD_N_HEADINGS : (heading + FD_N_HEADINGS - 1) % FD_N_HEADINGS;
            #ifdef FD_DEBUG_RU
                Serial.print(F("Run() heading: "));
                Serial.println(heading);
            #endif
            updateCourse = true;
        }
        if (vChange != steady) {
            vHeading = vChange == rising ? min(vHeading + 1, FD_N_VHEADINGS - 1) : max(vHeading - 1, 0);
            #ifdef FD_DEBUG_RU
                Serial.print(F("Run() vHeading: "));
                Serial.println(vHeading);
            #endif
            updateCourse = true;
        }
        turnClock = now;
        if (updateCourse) {
            go();
        }
    }
    multiStepper.run();
}

void FlyingDiver::setHomePosition() {
    for (int8_t n = 0; n < 4; n++) {
        stepper[n].setCurrentPosition(mmToSteps(cableLength(n, Point3D {FD_HOME_X, FD_HOME_Y, FD_HOME_Z})));
    }
    #ifdef FD_DEBUG_SH
        Serial.print(F("setHomePosition cables: "));
        for (int8_t n = 0; n < 4; n++) {
            Serial.print(cLength(n));
            Serial.print(n == 3 ? F("\n") : F(", "));
        }
    #endif
    heading = 0;
    turning = straight;
    vHeading = FD_VHEADING_STEADY;
    vChange = steady;
    isCalibrated = true;
}

void FlyingDiver::setEnable(bool en) {
    for (int8_t n = 0; n < 4; n++) {
        if (en) {
            stepper[n].enableOutputs();
        } else {
            stepper[n].disableOutputs();
        }
    }
    driversEnabled = en;
}

int8_t FlyingDiver::moveTo(Point3D p) {
    #ifdef FD_DEBUG_MT
        Serial.println(F("Entered moveTo()"));
    #endif
    if (!isCalibrated || !driversEnabled) {
        #ifdef FD_DEBUG_MT
            Serial.println(F("System not ready."));
        #endif
        return FD_NOT_READY;
    }
    if (!isInBounds(p)) {
        #ifdef FD_DEBUG_MT
            Serial.println(F("Asked to move out of bounds."));
        #endif
        return FD_OOB;
    }

    long newLength[4];
    for (int8_t n = 0; n < 4; n++) {
        newLength[n] = mmToSteps(cableLength(n, p));
        #ifdef FD_DEBUG_MT
            Serial.print(F("newLength["));
            Serial.print(n);
            Serial.print(F("]: "));
            Serial.print(newLength[n]);
            Serial.print(n == 3 ? F("\n") : F(", "));
        #endif
    }
    multiStepper.moveTo(newLength);
    return FD_OK;
}

int8_t FlyingDiver::moveBy(Point3D delta) {
    Point3D cur = where();
    #ifdef FD_DEBUG_MB
        Serial.print(F("moveBy: ("));
        Serial.print(delta.x);
        Serial.print(F(", "));
        Serial.print(delta.y);
        Serial.print(F(", "));
        Serial.print(delta.z);
        Serial.print(F(") from ("));
        Serial.print(cur.x);
        Serial.print(F(", "));
        Serial.print(cur.y);
        Serial.print(F(", "));
        Serial.print(cur.z);
        Serial.print(F(") to ("));
        Serial.print(cur.x + delta.x);
        Serial.print(F(", "));
        Serial.print(cur.y + delta.y);
        Serial.print(F(", "));
        Serial.print(cur.z + delta.z);
        Serial.println(F(")"));
    #endif
    return moveTo(Point3D {cur.x + delta.x, cur.y + delta.y, cur.z + delta.z});
}

int8_t FlyingDiver::moveHome() {
    #ifdef FD_DEBUG_MH
        Serial.println(F("Returning home."));
    #endif
    heading = 0;
    turning = straight;
    vHeading = FD_VHEADING_STEADY;
    vChange = steady;
    return moveTo(Point3D {FD_HOME_X, FD_HOME_Y, FD_HOME_Z});
}

int8_t FlyingDiver::turn(turnDir d) {
    turning = d;
    if (isRunning()) {
        return go();
    }
    return FD_OK;
}

int8_t FlyingDiver::vTurn(vert h) {
    vChange = h;
    if (isRunning()) {
        return go();
    }
    return FD_OK;
}

int8_t FlyingDiver::go(){

    if (!isCalibrated) {
        #ifdef FD_DEBUG_GO
            Serial.println(F("System not calibrated."));
        #endif
        return FD_NOT_READY;
    }

    Point3D here = where();
    // Figure y and z intercepts
    float by = here.y - (slope[heading] * here.x);
    float bz = here.z - (vSlope[vHeading] * here.x);

    // Assume we'll aim to hit the left or right wall (i.e., along the x axis)
    float x = xIsRising(heading) ? FD_X_SIZE - FD_X_SAG : FD_X_SAG;
    float y = slope[heading] * x + by;
    float z = vSlope[vHeading] * x + bz;

    // If that would result in hitting the front or back, assume that's what we hit
    if (y < FD_Y_SAG || y > FD_Y_SIZE - FD_Y_SAG) {
        y = y < FD_Y_SAG ? FD_Y_SAG : FD_Y_SIZE - FD_Y_SAG;
        x = (y - by) / slope[heading];
        z = vSlope[vHeading] * x + bz;
    }

    //If that would result in hitting the top or bottom, go for that, instead.
    if (z < FD_Z_SAG || z > FD_Z_SIZE - FD_Z_SAG) {
        z = z < FD_Z_SAG ? FD_Z_SAG : FD_Z_SIZE - FD_Z_SAG;
        x = (z - bz) / vSlope[vHeading];
        y =  slope[heading] * x + by;
    }
    #ifdef FD_DEBUG_GO
        Serial.print(F("Aiming to hit "));
        Serial.print(x == FD_X_SAG ? F("left") :
            x == FD_X_SIZE - FD_X_SAG ? F("right") :
            y == FD_Y_SAG ? F("front") :
            y == FD_Y_SIZE - FD_Y_SAG ? F("back") :
            z == FD_Z_SAG ? F("bottom") : F("top"));
        Serial.print(F(". Headings h: "));
        Serial.print(heading);
        Serial.print(F(", v: "));
        Serial.print(vHeading);
        Serial.print(F(". Going toward ("));
        Serial.print((int32_t)x);
        Serial.print(F(", "));
        Serial.print((int32_t)y);
        Serial.print(F(", "));
        Serial.print((int32_t)z);
        Serial.println(F(")"));
    #endif
    return moveTo(Point3D {(int32_t)x, (int32_t)y, (int32_t)z});
}

void FlyingDiver::stop() {
    for (int8_t n = 0; n < 4; n++) {
        stepper[n].stop();
        stepper[n].moveTo(stepper[n].currentPosition());
    }
}

Point3D FlyingDiver::where() {
    int32_t c[4] = {cLength(0), cLength(1), cLength(2), cLength(3)};
    return where(c);
}

Point3D FlyingDiver::where(int32_t c[4]) {

    // From Weisstein, Eric W. "Sphere-Sphere Intersection." From 
    // MathWorld--A Wolfram Web Resource. 
    // https://mathworld.wolfram.com/Sphere-SphereIntersection.html the x
    // coordinate of the circle that is the intersection of two spheres, one
    // which is centered at the origin and has radius R and the other of which 
    // has radius r and is centered on the x axis at distance d along is given
    // by
    //      x = (d^2 - r^2 + R^2) / (2 * d) = d / 2 + (R^2 - r^2) / (2 * d)
    //
    // So we have
    int32_t x = (float)FD_X_SIZE / 2.0 + (float)(c[0] * c[0] - c[1] * c[1]) / (float)(FD_X_SIZE * 2) + 0.4999;
    // where the +0.4999 and the FD_X_SIZE in the numerators give us rounding instead of truncation
    //
    // Similarly for y
    int32_t y = (float)FD_Y_SIZE / 2.0 + (float)(c[0] * c[0] - c[2] * c[2]) / (float)(FD_Y_SIZE * 2) + 0.4999;

    // From the formula for a sphere and knowing x and y we can use any of the 
    // c[0 .. 3] to calculate z. Since we're dealing in integers, 
    // we'll use the largest for the best resolution.
    float r = c[0];
    float dx = 0.0;
    float dy = 0.0;
    for (int8_t n = 1; n < 4; n++) {
        if (c[n] > r) {
            r = c[n];
            dx = n % 2 ? FD_X_SIZE : dx;
            dy = n > 1 ? FD_Y_SIZE : dy;            
        }
    }
    #ifdef FD_DEBUG_W
        Serial.print(F("where() r: "));
        Serial.print(r);
        Serial.print(F(", x: "));
        Serial.print(x);
        Serial.print(F(", dx: "));
        Serial.print(dx);
        Serial.print(F(", y: "));
        Serial.print(y);
        Serial.print(F(", dy: "));
        Serial.println(dy);
        Serial.print(F("r^2: "));
        Serial.print(r * r);
        Serial.print(F(", (x - dx)**2: "));
        Serial.print((x - dx) * (x - dx));
        Serial.print(F(", (y - dy)**2: "));
        Serial.println((y - dy) * (y - dy));
    #endif
    int32_t z = (FD_Z_SIZE - sqrt(r * r - (x - dx) * (x - dx) - (y - dy) * (y - dy))) + 0.4999;

    return Point3D {x, y, z};
}

Point3D FlyingDiver::where(Point3D p) {
    int32_t cable[4];
    #ifdef FD_DEBUG_W
        Serial.print(F("where(Point3D) cable: ("));
    #endif
    for (int8_t n = 0; n < 4; n++) {
        cable[n] = cableLength(n, p);
        #ifdef FD_DEBUG_W
            Serial.print(cable[n]);
            Serial.print(n == 3 ? F(")\n") : F(", "));
        #endif
    }
    return where(cable);
}

bool FlyingDiver::isRunning() {
    #ifdef FD_DEBUG_IR
        Serial.print(F("isRunning: "));
        for (int8_t n = 0; n < 4; n++) {
            if (stepper[n].isRunning()) {
                Serial.print(F("Y"));
            } else {
                Serial.print(F("n"));
            }
        }
        Serial.print(F("\n"));
    #endif
    return stepper[0].isRunning() || stepper[1].isRunning() ||
           stepper[2].isRunning() || stepper[3].isRunning();
}
