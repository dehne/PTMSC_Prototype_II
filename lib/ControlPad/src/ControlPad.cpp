/****
 * 
 * This file is a portion of the package ControlPad, a library that provides 
 * an Arduino sketch with the representation of a control pad consisting of
 * a joystick and two button switches. See ControlPad.h for details.
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

#include "ControlPad.h"

ControlPad::ControlPad(int8_t pinLR, int8_t pinFS, int8_t pinUP, int8_t pinDN) {
    _pinLR = pinLR;
    _pinFS = pinFS;
    _pinUP = pinUP;
    _pinDN = pinDN;
    for(int8_t i = 0; i <= CP_S_MAX; i++) {
        _handler[i] = NULL;
    }
}

void ControlPad::begin() {
    pinMode(_pinUP, INPUT);
    pinMode(_pinDN, INPUT);
}

void ControlPad::run() {
    unsigned long now = max(millis(), 1);   // 0 is special

    // upSw
    bool cur = digitalRead(_pinUP);
    if (_upChg == 0 && _upSw == cur) {   // The pin is active high so == is a change
        _upChg = now;
    } else if (now - _upChg > CP_DEBOUNCE_MS) {
        _upChg = 0;
        if (_upSw == cur) {
            _upSw = !cur;
            if (_upSw) {
                if (_handler[upPressed] != NULL) {
                    (*_handler[upPressed])();
                }
            } else if (_handler[upReleased] != NULL) {
                (*_handler[upReleased])();
            }
        }
    }

    // dnSw
    cur = digitalRead(_pinDN);
    if (_dnChg == 0 && _dnSw == cur) {   // Again, active high
        _dnChg = now;
    } else if (now - _dnChg > CP_DEBOUNCE_MS) {
        _dnChg = 0;
        if (_dnSw == cur) {
            _dnSw = !cur;
            if (_dnSw) {
                if (_handler[downPressed] != NULL) {
                    (*_handler[downPressed])();
                }
            } else if (_handler[downReleased] != NULL) {
                (*_handler[downReleased])();
            }
        }
    }

    // goFw
    cur = analogRead(_pinFS) < CP_ST_LIMIT;
    if (cur != _goFw) {
        if (cur) {
            if (_handler[toForward] != NULL) {
                (*_handler[toForward])();
            }
        } else if (_handler[toStop] != NULL) {
            (*_handler[toStop])();
        }
        _goFw = cur;
    }

    // goRt
    int16_t curLR = analogRead(_pinLR);
    cur = curLR < CP_R_LIMIT;
    if (cur != _goRt) {
        if (cur) {
            if (_handler[toRight] != NULL) {
                (*_handler[toRight])();
            }
        } else if (_handler[toNeutral] != NULL) {
            (*_handler[toNeutral])();
        }
        _goRt = cur;
    }

    // GoLt
    cur = curLR > CP_L_LIMIT;
    if (cur != _goLt) {
        if (cur) {
            if (_handler[toLeft] != NULL) {
                (*_handler[toLeft])();
            }
        } else if (_handler[toNeutral] != NULL) {
            (*_handler[toNeutral])();
        }
        _goLt = cur;
    }
}

void ControlPad::attachHandler(cpStateChange s, controlPadHandler h) {
    _handler[s] = h;
}