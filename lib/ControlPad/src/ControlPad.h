/****
 * 
 * This file is a portion of the package ControlPad, a library that provides 
 * an Arduino sketch with the representation of a control pad consisting of
 * a joystick and two button switches. 
 * 
 * The joystick is an analog device composed of two pots (forward-backward 
 * and left-right) together with a switch that can be activeted by pressing 
 * down on the control. The switch and the analog nature of the device is not 
 * used here. Instead we only use it to extract signals that detect when the 
 * user moves the stick from neutral to forward, forward to neutral, neutral 
 * to left, left to neutral, neutral to right, or right to neutral. Note the 
 * the axes are independent of one another. For example, it's possible to be 
 * in the forward state on the first axis while still in the neutral position 
 * on the second. The two buttons are handled similarly -- we look for state 
 * changes from pressed to not pressed and not pressed to pressed. The switch 
 * state changes are debounced.
 * 
 * When a state change is detected, the corresponding user-provided handler 
 * function, if any, is invoked. User-provided handler functions are
 * registered using the addHandler() member function. There can be at most 
 * one handler per state change; adding a handler for a state change for which 
 * there is already one provided replaces the old handler with the new one.
 * 
 * To make all this work, include a call to the ControlPad's run member 
 * function in the sketch's loop() function. And invoke it often.
 * 
 *****
 * 
 * ControlPad V0.1, April 2020
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
#ifndef CONTROL_PAD
    #define CONTROL_PAD

    #if ARDUINO >= 100
        #include "Arduino.h"
    #else
         #include "WProgram.h"
    #endif

    // Various hardware-related constants
    #define CP_DEBOUNCE_MS  (10)        // How long (ms) it takes a switch to settle down
    #define CP_ST_LIMIT     (250)       // Reading on FS pot less than this implies goFw == true
    #define CP_L_LIMIT      (830)       // Reading on LR pot more than this implies goL == true
    #define CP_R_LIMIT      (550)       // Reading on LR pot less than this implies goR == true

    // The state changes ControlPad detects
    enum cpStateChange : int8_t {toForward, toStop, toLeft, toNeutral, toRight, 
        downPressed, downReleased, upPressed, upReleased, CP_S_MAX = upReleased};

    extern "C" {
        // User-supplied handler functions always follow the signature: void cmd(void);
        typedef void(*controlPadHandler) (void);
    }


    class ControlPad {
        public:
            /**
             * 
             * Make a new ControlPad object. 
             * 
             * Parameters
             *  pinLR       The analog pin to which the left-right pot is attached
             *  pinFS       The analog pin to which the forward-stop pot is attached
             *  pinUP       The digital pin to which the up button is attached
             *  pinDN       The digital pin to which the down button is attached.
             * 
             **/
            ControlPad(int8_t pinLR, int8_t pinFS, int8_t pinUP, int8_t pinDN);

            /**
             * 
             * Initialize ControlPad Object. Invoke this in the sketch's setup()
             * 
             **/
            void begin();

            /**
             * 
             * Look at the control pad hardware, detect any state changes, and
             * invoke the requisite user supplied handler functions.
             * 
             **/
            void run();

            /**
             * 
             * Add a handler for the given state. When the control pad state 
             * changes to that state, the handler is invoked. Adding a handler 
             * for a state that already has a handle, replaces the old one 
             * with the new one. If you add NULL as a handler, the existing 
             * one is removed, and no new one is asdded.
             * 
             * Parameters
             *  s       The ControlPadState that the supplied handler is for
             *  h       The handler function, a void function with no 
             *          parameters
             * 
             **/
            void attachHandler(cpStateChange s, controlPadHandler h);

        private:
            controlPadHandler _handler[CP_S_MAX + 1];       // The handlers
            bool _goLt = false;                             // State of the control pad controls
            bool _goRt = false;                             // Left, right, forward
            bool _goFw = false;
            bool _upSw = false;                             // State of Up and Down buttons; true = pressed
            bool _dnSw = false;
            unsigned long _upChg = 0;                       // millis() when we last saw button and upSw disagree
            unsigned long _dnChg = 0;                       // millis() when we last saw button ans dnSw disagree
            int8_t _pinLR;
            int8_t _pinFS;
            int8_t _pinUP;
            int8_t _pinDN;

    };
#endif