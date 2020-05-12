/****
 * 
 * This file is a portion of the package FlyingPlatform, a library that 
 * provides an Arduino sketch with the ability to control four stepper motors 
 * that operate a "flying camera" setup in which a platform is connected by 
 * cables to four stepper-motor-controlled winches. By judiciously reeling in 
 * and letting out the cables, the platform can be moved around in three 
 * space.
 * 
 * The origin of the coordinate system is at the lower, front, left corner of 
 * a rectangular prism with its faces parallel to the axes. This prism is 
 * called the "flying space." The +x direction is to the to the right, +y is 
 * toward the back, and +z is up. The measurements are all in steps. A step is 
 * the amount of cable a winch reels in or lets out when its stepper motor is 
 * moved by one step. The winches are designed so that the length of a step is 
 * constant. The winches are located at the four top corners of the flying 
 * space.
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
#ifndef FLYING_PLATFORM
    #define FLYING_PLATFORM

    #if ARDUINO >= 100
        #include "Arduino.h"
    #else
         #include "WProgram.h"
    #endif

    /**
     * 
     * Uncomment to turn on debugs
     * 
     **/
    //#define FP_DEBUG_ISR        // ISR (A bunch of it is in run())
    //#define FP_DEBUG_RU         // run()
    //#define FP_DEBUG_MT         // moveTo()
    //#define FP_DEBUG_MB         // moveBy()
    //#define FP_DEBUG_NT         // newTarget()
    //#define FP_DEBUG_W          // where()
    #define FP_DEBUG_LED        // Use LED on pin A5 to show stepper(s) running

    /**
     * 
     * Some compile-time constants
     * 
     **/
    #define FP_MAX_SPEED        (800)       // Default maximum speed (steps/sec)
    #define FP_BATCH_STEPS      (128)       // Default size of batches (steps)
    #define FP_N_HHEADINGS      (40)        // Number of horizontal headings
    #define FP_N_VHEADINGS      (11)        // Number of vertical headings
    #define FP_LEVEL            (5)         // The vertical heading for fp_level
    #define FP_TURN_INTERVAL    (250000)    // Heading change interval (in μs)
    #define FP_MAX_JITTER       (50)        // Maximum jitter (μs) in dispatching a step
    #define FP_STARTING_COUNT   (0X40)      // OCR0A count value for first interrupt


    /**
     * 
     * Macros
     * 
     **/
    #define fp_xIsRising(hh)     (hh < FP_N_HHEADINGS / 4 || hh >  3 * (FP_N_HHEADINGS / 4))

    /**
     * 
     * Some enums for readability
     *
     **/
    enum fp_hTurns : byte {fp_left, fp_straight, fp_right};
    enum fp_vTurns : byte {fp_falling, fp_level, fp_rising};

    /**
     * 
     * Representation of a point in the flying space
     * 
     **/
    struct fp_Point3D {
        long x;
        long y;
        long z;
    };


    /**
     * 
     * Codes returned by various member functions
     * 
     * fp_ok:   Things went well
     * fp_oob:  The resulting move would have moved the platform outside of 
     *          the safe area. No movement was made.
     * fp_ncp:  The current position has not been set, so the platform can't 
     *          be moved. 
     * fp_dis:  The steppers are disabled, so the platform can't be moved.
     * fp_mov:  The operation can't be done because a move is underway.
     * fp_nom:  The operation can't be done because no move is underway.
     * 
     **/
    enum fp_return_code : byte{fp_ok, fp_oob, fp_ncp, fp_dis, fp_mov, fp_nom};

    class FlyingPlatform {
        public:
            /**
             * 
             * Make a new FlyingPlatform object representing a flying platform 
             * controlled by four stepper motors driven by TB6600 motor 
             * controllers driving winches located in the specified places. 
             * The hoist points for the four cables, measured in steps, are 
             * located at:
             *      cable 0:    (0, 0, spaceHeight)
             *      cable 1:    (spaceWidth, 0, spaceHeight)
             *      cable 2:    (0, spaceDepth, spaceHeight)
             *      cable 3:    (spaceWidth, spaceDepth, spaceHeight)
             * 
             * Parameters:
             *  pin0D       The direction pin for stepper 0. High means 
             *              lengthen cable.
             *  pin0P       The step pin for stepper 0. Normally low. Pulse 
             *              to move one step in direction indicated by pin0D.
             *  pin1D       Same but for stepper 1
             *  pin1P
             *  pin2D       Same but for stepper 2
             *  pin2p
             *  pin3D       Same but for stepper 3
             *  pin4P
             *  pinEn       Enable pin -- common to all steppers. Active low.
             *  spaceWidth  The width, in steps, of the space in which the 
             *              flyer operates
             *  spaceDepth  Its depth
             *  spaceHeight Its height
             * 
             **/
            FlyingPlatform(
                byte pin0D, byte pin0P, 
                byte pin1D, byte pin1P, 
                byte pin2D, byte pin2P, 
                byte pin3D, byte pin3P,
                byte pinEn,
                long spaceWidth, long spaceDepth, long spaceHeight);

            /**
             * 
             * Do the final initialization. Put this in the sketch's setup() 
             * function.
             * 
             **/
            void begin();

            /**
             * 
             * Have the FlyerPLATFORM object do its thing running the motors. 
             * Put an invocation of this in the sketch's loop() function.
             * 
             * Returns true if any of the motors still have steps to go; false 
             * otherwise.
             * 
             **/
            bool run();

            /**
             * 
             * Set the maximum speed, in steps per second, at which we're 
             * allowed to emit step pulses. Defaults to 800.
             * 
             **/
            void setMaxSpeed(float speed);

            /**
             * 
             * Set the safety margins, the distances from each of the sides of 
             * the flying space from which the platform is excluded. Setting
             * these keeps the platform from accidententally crashing into 
             * something and from overstressing the mechanism. This is 
             * particularly important for the top of the flying space since 
             * getting near to the top puts considerable strain on the cables.
             * Before this is called, the platform can be positioned at any 
             * location in the flying space.
             * 
             * Parameters
             *  leftMargin      The platform's x coordinate must be more than 
             *                  this.
             *  rightMargin     The platform's y coordinate must be less than
             *                  this.
             *  frontMargin     Same for minimum y
             *  backMargin      Same for maximum y
             *  bottomMargin    Same for minimum z
             *  topMargin       Same for maximum z
             * 
             * Returns fp_ok if all went well, fp_oob if the specified margins 
             * would cause the platform's current position to be out of 
             * bounds.
             * 
             **/
            fp_return_code setSafetyMargins(
                long leftMargin, long rightMargin, 
                long frontMargin, long backMargin, 
                long bottomMargin, long topMargin);

            /**
             * 
             * Set the batch size, the length, in steps, of of a batch of 
             * steps. A shorter integration interval causes the deviation 
             * from a straight line the path the platform takes to diminish, 
             * but it increases the computational load substantially. Maybe 
             * something about a cm in size. Defaults to 128 steps.
             * 
             **/
            void setBatchSize(long steps);

            /**
             * 
             * Set the current position to the given location in the flying 
             * space and reset the headings. That is, don't move anything, 
             * just assume that the platform is at the position specified, set 
             * the assumed length of the cables to match that, and set 
             * hHeading to 0, vHeading to FD_LEVEL.
             * 
             * Returns fp_ok if all went well, fp_oob if the specified 
             * position is out of bounds of the flying space, including the
             * safety margins. No change to the current location is made.
             * 
             **/
            fp_return_code setCurrentPosition (fp_Point3D tgt);

            /**
             * 
             * Enable the motor drivers. When enabled, the steppers are 
             * capable of stepping. Returns fp_ok.
             * 
             **/
            fp_return_code enableOutputs();

            /**
             * 
             * Disable the motor drivers. Returns fp_ok if successful, If a
             * move is underway, returns fp_mov and motors are not disabled.
             * 
             * When disabled, the steppers can't step, but can be turned by 
             * hand. Also, disabled motors don't draw much power or create 
             * heat, probably cutting down on wear and tear on the mechanism. 
             * On the other hand, when powered down, the motors don't hold 
             * position very well at all and will turn by themselves if under 
             * load.
             * 
             **/
            fp_return_code disableOutputs();

            /**
             * 
             * Start the motors running to move the platform from its current 
             * location to the position indicated. This is non blocking; the 
             * actual movement occurs over time ar run() is repeatedly 
             * invoked. If moveTo() is invoked while the platform is moving, 
             * its journey is rerouted from whatever it was to the new 
             * destination.
             * 
             * Returns 
             *      fp_ok   if successful. 
             *      fp_oob  if the specified position is outside the flying 
             *              space (including the safety margins). In this case 
             *              the platform will stop if it was moving or remain 
             *              at rest if it wasn't. 
             *      fp_ncp  if the current position hasn't been set. (No 
             *              movement happens.)
             *      fp_dis  if the motors are disabled. (No movement.)
             * 
             **/
            fp_return_code moveTo(fp_Point3D tgt);

            /**
             * 
             * Kick off the process of moving to tgt. This is non-blocking; the 
             * actual work occurs over time as run() and the ISR are repeatedly 
             * invoked. If moveTo() is invoked while the platform is moving, 
             * its journey is rerouted from whatever it was to the new 
             * destination.
             * 
             * Returns 
             *      fp_ok   if successful. 
             *      fp_oob  if the specified position is outside the flying 
             *              space (including the safety margins). In this case 
             *              the platform will stop if it was moving or remain 
             *              at rest if it wasn't. 
             *      fp_ncp  if the current position hasn't been set. (No 
             *              movement happens.)
             *      fp_dis  if the motors are disabled. (No movement.)
             * 
             **/

            fp_return_code moveBy(fp_Point3D delta);

            /**
             * 
             * Set the platform moving along its heading-defined path. The 
             * heading-defined path consists of its horizontal and vertical 
             * headings, hHeading and vHeading, and how those headings are
             * changing, hChange and vChange. 
             * 
             * The variable hChange can have values fp_left, fp_straight, or 
             * fp_right. When it is fp_left the hHeading decreases (modulo the 
             * number of hHeadings) once every FP_TURN_INTERVAL. If it is 
             * fp_right, it increases in the same way. If it is fp_straight, 
             * the hHeading doesn't change.
             * 
             * The variable vChange is similar but a little different. It can
             * have the values fp_rising, fp_level, or fp_falling. if it is 
             * fp_rising, the vHeading increases once every FP_TURN_INTERVAL 
             * until it reaches its maximum value. If it is fp_falling, it
             * decreases in the sam way until it reaches 0. If it is fp_level 
             * it either increases or decreases every FP_TURN_INTERVAL until 
             * it reaches FP_LEVEL, at which point it stops changing.
             *
             * Returns 
             *      fp_ok   if successful. 
             *      fp_ncp  if the current position hasn't been set. (No 
             *              movement happens.)
             *      fp_dis  if the motors are disabled. (No movement.)
             * 
             **/
            fp_return_code go();

            /**
             * 
             * Change how the platform moves in the x-y direction. You can 
             * only change direction while moving. If not moving nothing 
             * changes, but you get a fp_nom return code.
             * 
             * See go() for details on how turns work.
             * 
             * Parameter
             *      dir     The direction to turn: fp_left, fp_striaght or 
             *              fp_right
             * 
             * Returns 
             *      fp_ok   if successful. 
             *      fp_nom  if no move is underway
             * 
             **/
            fp_return_code turn(fp_hTurns dir);

            /**
             * 
             * Change how the platform moves in the z direction. You can 
             * only change direction while moving. If not moving nothing 
             * changes, but you get a fp_nom return code.
             * 
             * See go() for details on how turns work.
             * 
             * Parameter
             *      dir     The direction to turn: fp_rising, fp_level or
             *              fp_falling.
             * 
             * Returns 
             *      fp_ok   if successful. 
             *      fp_nom  if no move is underway
             * 
             **/
            fp_return_code turn(fp_vTurns dir);

            /**
             * 
             * Stop all the motors as quickly as possible.
             * 
             **/
            void stop();

            /**
             * 
             * Return the platform's current location in the flying space.
             * 
             * Returns fp_Point3D {-1, -1, -1} if current position hasn't 
             * been set.
             * 
             **/
            fp_Point3D where();

            /**
             * 
             * Returns true if any of the motors currently have steps to make; 
             * false otherwise.
             * 
             **/
            bool isRunning();

        private:

            /**
             * 
             * Based on current position and on hHeading and vHeading, 
             * return a fp_Point3D of the corresopnding target. The target is 
             * the point on one of the planes formed by the margins which we 
             * will eventually hit if we move from where we are along a 
             * straight path described by hHeading and vHeading.
             * 
             * NB: The code assumes that currentIsSet but, for efficiency 
             * doesn't check, so be careful calling it.
             * 
             **/
            fp_Point3D newTarget();

            byte enablePin;                     // Common pin to enable/disable motor drivers. Active LOW.

            fp_Point3D space;                   // The right, back, top corner of the flying space
            fp_Point3D marginsMin;              // Safety margins minimum values (left, front, bottom)
            fp_Point3D marginsMax;              // Safety margins maximum values (right, back, top)
            long batchSteps;                    // The length along the direction of motion, in steps, of a batch

            fp_Point3D target;                  // Where we're trying to go
            fp_Point3D source;                  // Where we're coming from
            float nextCableSteps[4];            // Cable lengths at the beginning of next batch of steps
            float dt;                           // Fraction of the move a batch is
            float t;                            // How far along we are in the move [0..1]

            bool newMove;                       // True if a new move has been set but not started
            bool stopping;                      // True if waiting for the current batch to finish so we can stop

            float maxSpeed;                     // Max speed in steps per second.
            bool currentIsSet;                  // Whether cableSteps[] has been initialized
            bool isEnabled;                     // Whether motor drivers are enabled

            /**
             * 
             * In addition to its position, the flying platform has horizontal 
             * (x-y) and vertical (z along the x-y path) headings.
             * hSlope is the list slopes (the m in y = mx + b) for the 40 
             * possible horizontal headings. Basically they are tan(heading) 
             * spaced equally. There are 40 of them so they are 9 degrees apart.
             * Note that they avoid an infinite slope; the code relies on this
             * 
             **/
            float hSlope[FP_N_HHEADINGS] =
            {0,             0.1583844403, 0.3249196962,  0.5095254495,  0.726542528, 
             1,             1.37638192,   1.962610506,   3.077683537,   6.313751515, 
             1.63312E+16,  -6.313751515, -3.077683537,  -1.962610506,  -1.37638192,
            -1,            -0.726542528, -0.5095254495, -0.3249196962, -0.1583844403, 
             0,             0.1583844403, 0.3249196962,  0.5095254495,  0.726542528, 
             1,             1.37638192,   1.962610506,   3.077683537,   6.313751515, 
             5.44375E+15,  -6.313751515, -3.077683537,  -1.962610506,  -1.37638192, 
             -1,           -0.726542528, -0.5095254495, -0.3249196962, -0.1583844403};
            int hHeading;                       // Which slope the diver is currently on
            fp_hTurns hChange;                  // How hHeading is changing fp_left, fp_straight, fp_right

            /**
             * 
             * vSlope is the list of slopes for the 11 possible diver 
             * vHeadings -- the m in dz = m * sqrt(dx**2 + dy**2).
             * They are tan(10 * (vHeading - 5)), so 10 degrees apart, with 
             * vHeading = 5 as fp_level
             * 
             **/
            float vSlope[FP_N_VHEADINGS] = 
            {-1.1917535930, -0.8390996312, -0.5773502692, -0.3639702343, -0.1763269807,
              0, // <-- vChange == fp_level
              0.1763269807,  0.3639702343,  0.5773502692,  0.8390996312,  1.191753593};
            int vHeading;                       // Which vSlope diver is currently on
            fp_vTurns vChange;                  // How diver's vertical direction is changing
            unsigned long turnMicros;           // micros() when horizontal and vertical turns changed
    };
#endif