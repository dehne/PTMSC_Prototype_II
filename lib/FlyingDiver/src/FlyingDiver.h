/****
 * 
 * This file is a portion of the package FlyingDiver, a library that provides 
 * an Arduino sketch with the ability to control the diver for the Port 
 * Townsend Marine Science Center's pinto abalone exhibit.
 * 
 * The diver in this exhibit is controlled using a mechanism like the "flying 
 * cameras" used by the NFL to provide overhead video of football games. The 
 * diver model is attached to four cables whose length is controlled by four 
 * stepper motors. The main job of this library's FlyingDiver object is to 
 * operate those four stepper motors to produce the diver's motion in the 
 * exhibit. The member functions provide access for the Arduino sketch to 
 * cause the FlyingDiver to move the diver around the exhibit.
 * 
 * About the coordinate system and measurements
 * 
 * The origin of the coordinate system is at the lower, front, left corner. 
 * The +x direction is to the to the right, +y is toward the back, and +z is 
 * up. Generally measurements are in millimeters, but the stepper motors work
 * in steps, so when talking to them we use steps
 * 
 *****
 * 
 * FlyingDiver V0.2, April 2020
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
#ifndef FLYING_DIVER
    #define FLYING_DIVER

    #if ARDUINO >= 100
        #include "Arduino.h"
    #else
         #include "WProgram.h"
    #endif

    #include "AccelStepper.h"
    #include "MultiStepper.h"

    // Uncomment to turn on debugging messages
    //#define FD_DEBUG_CL   // cableLength()
    #define FD_DEBUG_RU     // run()
    //#define FD_DEBUG_MT   // moveTo()
    //#define FD_DEBUG_MB   // moveBy()
    //define FD_DEBUG_SH    // setHomePosition()
    //#define FD_DEBUG_MH   // moveHome()
    #define FD_DEBUG_W    // where() (all overloads)
    //#define FD_DEBUG_IR   // isRunning()
    #define FD_DEBUG_GO   // go()

    // Various stepper motor constants and conversions
    #define FD_MAX_SPEED        (200.0)           // in steps / sec
    #define FD_MM_PER_REV       (31.51)           // Millimeters of wire travel per revolution
    #define FD_STEPS_PER_REV    (400.0)           // What it sounds like
    #define FD_STEPS_PER_MM     (FD_STEPS_PER_REV / FD_MM_PER_REV)
    #define mmToSteps(mm)       ((int32_t)((((mm) * FD_STEPS_PER_REV) / FD_MM_PER_REV) + 0.5))
    #define stepsToMm(steps)    ((int32_t)(((steps) / FD_STEPS_PER_MM) + 0.5))

    // Overall size in mmm measures between hoist points (for x and y) and 
    // the floor and the hoist point (for z). Note: Those are the letter "l" 
    // at the end of the values to force them to be int32_t ("long").
    #define FD_X_SIZE           (950l)
    #define FD_Y_SIZE           (565l)
    #define FD_Z_SIZE           (753l)


    // How close to the edges of the volume of the prism the diver can get
    #define FD_X_SAG            (25)
    #define FD_Y_SAG            (20)
    #define FD_Z_SAG            (FD_Z_SIZE - FD_HOME_Z)

    // Home position -- i.e., the position at which a calibration places the 
    // diver
    #define FD_HOME_X           (475)
    #define FD_HOME_Y           (283)
    #define FD_HOME_Z           (650)

    // Shorthand for retrieving the current length of a cable
    #define cLength(c)          (stepsToMm(stepper[c].currentPosition()))

    // Return codes for movement member functions
    #define FD_OK               (0)         // Success
    #define FD_OOB              (1)         // Asked to move out of bounds
    #define FD_NOT_READY        (2)         // System is not ready for movement
    #define FD_MOVING           (3)         // Already moving

    // Related to relative (heading-based) movement
    #define FD_N_HEADINGS       (40)        // Number of different headings
    #define FD_N_VHEADINGS      (11)        // Number of different vHeadings
    #define FD_VHEADING_STEADY  (5)         // The vHeading that's not rising or falling
    #define FD_XZ_SLOPE         (1.0/5.0)   // Slope in x-z plane of rising / falling
    #define FD_TURN_MS          (500)       // Time (ms) between changes to heading and vHeading
    enum vert : int8_t{rising, steady, falling};
    enum turnDir : int8_t{left, straight, right};

    // Shorthand for whether given heading has x rising (or falling)
    #define xIsRising(h)  ((h < (FD_N_HEADINGS / 4)) || h >= 3 * FD_N_HEADINGS / 4)

    // Convenient package for a 3-D point
    struct Point3D {
        int32_t x;
        int32_t y;
        int32_t z;
    };

    class FlyingDiver {
        public:
            /****
             * 
             * Make a new FlyingDiver object
             * 
             * Parameters 
             *      pinxy   x = stepper number; y = D => Direction pin, P => Pulse pin
             *      pinEn   the common enable pin shared by all steppers
             * 
             ****/
            FlyingDiver(
                int8_t pin0D = 2, int8_t pin0P = 3,         // The direction and pulse pins
                int8_t pin1D = 4, int8_t pin1P = 5,         // for the four stepper motors
                int8_t pin2D = 6, int8_t pin2P = 7,         // and the common enable pin they
                int8_t pin3D = 11, int8_t pin3P = 12,       // all share
                int8_t pinEn = 13);

            /**
             * 
             * run(): Invoke this member function in the sketch's loop() 
             * function as often as possible. It updates the stepper motors to 
             * make things move as needed.
             * 
             **/
            void run();

            /**
             * 
             * Assume the current position is the home position, i.e. the one 
             * that the diver goes to at the end of claibration. This is 
             * useful when "home" is set manually in lieu of doing a
             * calibration run.
             * 
             **/
            void setHomePosition();

            /**
             * 
             * Enable/disable stepper motor drivers
             * 
             * Parameter
             *  en      false => disable, true => enable
             * 
             **/
            void setEnable(bool en);

            /**
             * 
             * Move, in a straight line, from where we are to a specified 
             * point.
             * 
             * Parameter
             *      p   The specified point
             * 
             * Returns FD_OK if successful, FD_OOB if the specified point is 
             * outside the volume the diver is allowed to visit, FD_NOT_READY 
             * if the system is not ready for motion or FD_MOVING if the 
             * system is already in motion.
             * 
             **/
            int8_t moveTo(Point3D p);

            /**
             * 
             * Move, in a straight line, from where we are by the amount
             * specified by delta.
             * 
             * Parameter
             *      delta   The amount by which to move in x, y, and z
             * 
             * Returns FD_OK if successful, FD_OOB if the specified point is 
             * outside the volume the diver is allowed to visit, FD_NOT_READY 
             * if the system is not ready for motion or FD_MOVING if the 
             * system is already in motion.
             * 
             ***/
            int8_t moveBy(Point3D delta);

            /**
             * 
             * Move, in a straight line from where we are to the home position 
             * 
             * Returns FD_OK if successful, FD_NOT_READY if the system is not 
             * ready for motion.
             * 
             **/
            int8_t moveHome();

            /**
             * 
             * Set the diver's turning direction to the given turnDir. If the 
             * direction is turnDir {straight} turning stops. If it is
             * turnDir {left} or {right} turning in that direction continues
             * until it is stopped.
             * 
             * Returns FD_OK if successful, FD_NOT_READY if the system is not 
             * ready for motion.
             * 
             **/
            int8_t turn(turnDir t);

            /**
             * 
             * Set the diver's vertical turning direction to the given 
             * direction. Once set to rising or falling, turning continues 
             * until stopped by vTurn(vert {steady}) or until the limit
             * for rising or falling is achieved.
             * 
             * Returns FD_OK if successful, FD_NOT_READY if the system is not 
             * ready for motion.
             * 
             **/
            int8_t vTurn(vert h);

            /**
             * 
             * Move the diver along its current heading including its vertical 
             * direction until it bumps into a barrier or gets told to stop
             * or change its heading or vertical direction.
             * 
             * Returns FD_OK if successful, FD_NOT_READY if the system is not 
             * ready for motion.
             * 
             **/
            int8_t go();

            /**
             * 
             * Stop all motion as soon as possible.
             * 
             **/
            void stop();

            /**
             * 
             * Return a Point3D containing the diver's current location
             * 
             **/
            Point3D where();

            /**
             * 
             * Returns Point3D corresponding to the specified cable lengths
             * 
             * Parameter
             * c[4] The lengths of the four cables
             * 
             **/
            Point3D where(int32_t c[4]);

            /**
             * 
             * Test routine to see if cableLength and where(cable) agree.
             * p is fed to cableLength() to generate a set of cables which
             * is then fed through where(cable) to get the return value.
             * 
             * Parameter
             * Point3D  p
             * 
             * Returns Point3D
             * 
             **/
            Point3D where(Point3D p);

            /**
             * 
             * Returns true if any of the steppers are running; false otherwise
             * 
             **/
            bool isRunning();
            
        private:
            AccelStepper stepper[4];            // The four steppers
            MultiStepper multiStepper;          // The MultiStepper instance that manages the steppers
            #if defined(FD_DEBUG_MT) || defined(FD_DEBUG_GO)
                unsigned long startMillis[4];   // millis() when stepper started
                bool started[4];                // true if we started stepper
            #endif

            // slope is the list slopes (the m in y = mx + b) for the 40 possible diver headings
            // Basically they are tan(heading) spaced equally. There are 40 of them so they are
            // 9 degrees apart.
            float slope[FD_N_HEADINGS] =
            {0,             0.1583844403, 0.3249196962,  0.5095254495,  0.726542528, 
             1,             1.37638192,   1.962610506,   3.077683537,   6.313751515, 
             1.63312E+16,  -6.313751515, -3.077683537,  -1.962610506,  -1.37638192,
            -1,            -0.726542528, -0.5095254495, -0.3249196962, -0.1583844403, 
             0,             0.1583844403, 0.3249196962,  0.5095254495,  0.726542528, 
             1,             1.37638192,   1.962610506,   3.077683537,   6.313751515, 
             5.44375E+15,  -6.313751515, -3.077683537,  -1.962610506,  -1.37638192, 
             -1,           -0.726542528, -0.5095254495, -0.3249196962, -0.1583844403};
            int16_t heading = 0;                // Which slope the diver is currently on
            turnDir turning = straight;         // How heading is changing left, straight, right
            // vSlope is the list of slopes (the m in z = mx + b)for the 11 possible diver vHeadings
            // They are tan(10 * (vHeading - 5)), so 10 degrees apart, with vHeading = 5 as steady
            float vSlope[FD_N_VHEADINGS] = 
            {-1.1917535930, -0.8390996312, -0.5773502692, -0.3639702343, -0.1763269807,
              0, // <-- vChange = steady;
              0.1763269807,  0.3639702343,  0.5773502692,  0.8390996312,  1.191753593};
            int16_t vHeading = FD_VHEADING_STEADY;  // Which vSlope diver is currently on
            vert vChange = steady;              // How diver's vertical direction is changing
            unsigned long turnClock = 0;        // millis() when horizontal and vertical turns changed
            bool driversEnabled = true;         // Whether the motor drivers are enabled
            bool isCalibrated = false;          // Whether calibration is done
    };
#endif