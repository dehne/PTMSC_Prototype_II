/****
 * PTMSCPrototype V0.2, April 2020
 * 
 * Drive the pinto abalone exhibition prototype simulating a scuba diver doing
 * abalone outplanting using a "flying camera" mechanism simiar in principle 
 * to the cameras on four cables the NFL uses to "fly" over the field 
 * (technically called a "cable-driven parallel robot"). The prototype is 
 * intended to work out the kinks in the mechanism and its control before 
 * building the real thing. Here the software is controlled using a 
 * linux-like command interface delivered over the Arduino Serial stream or 
 * via a control pad specially made for this.
 * 
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

#include <Arduino.h>
#include "FlyingPlatform.h"
#include "UserInput.h"
#include "ControlPad.h"

#define DEBUG                       // Uncomment to turn on debugging messages

// For the stepper motors
#define PIN_0D        (2)
#define PIN_0P        (3)
#define PIN_1D        (4)
#define PIN_1P        (5)
#define PIN_2D        (6)
#define PIN_2P        (7)
#define PIN_3D        (11)
#define PIN_3P        (12)
#define PIN_EN        (13)
#define MAX_SPEED     (700)         // In steps per second

// Physical size of flying space (mm) measured from floor and between hoist points
#define SIZE_X        (950)
#define SIZE_Y        (565)
#define SIZE_Z        (753)

// Safety margins (mm) for left, right, front, back, down and up
#define MARGIN_L      (25)
#define MARGIN_R      (25)
#define MARGIN_F      (20)
#define MARGIN_B      (20)
#define MARGIN_D      (20)
#define MARGIN_U      (20)

// Home location
#define HOME_X        (474)
#define HOME_Y        (283)
#define HOME_Z        (650)

// Conversion between mm and steps. (The FlyingPlatform deals exclusively in steps.)
#define STEPS_PER_REV (800.0)       // What it sounds like
#define MM_PER_REV (31.51)          // Millimeters of wire travel per revolution
#define STEPS_PER_MM  (STEPS_PER_REV / MM_PER_REV)  // For 400 / 31.51 that's 12.694
#define mmToSteps(mm)       ((long)((mm) * STEPS_PER_MM))
#define stepsToMm(steps)    ((long)((steps) / STEPS_PER_MM))

// For the control pad
#define PIN_UP        (9)           // Pin to which "Go down" switch is connected; active low
#define PIN_DN        (10)          // Pin to which "Go up" switch is connected; active low
#define PIN_FS        (A1)          // Pin to which "Go forward or Stop" pot  is connected
#define PIN_LR        (A0)          // Pin to which "Go left, straight or right" pot is connected

// Misc compile-time definitions
#define BANNER          F("PTMSC Prototype v 0.2")

// Global variables
FlyingPlatform diver {
  PIN_0D, PIN_0P, PIN_1D, PIN_1P, 
  PIN_2D, PIN_2P, PIN_3D, PIN_3P, 
  PIN_EN, 
  mmToSteps(SIZE_X), mmToSteps(SIZE_Y), mmToSteps(SIZE_Z)};
UserInput ui {Serial};
ControlPad cp {PIN_LR, PIN_FS, PIN_UP, PIN_DN};

/**
 * 
 * Interpret FlyingPlatform return codes to Serial
 * 
 **/
void interpretRc(int8_t rc) {
  switch (rc)   {
    case fp_ok:
      Serial.println(F("Ok."));
      break;
    case fp_oob:
      Serial.println(F("Result would be out of bounds."));
      break;
    case fp_ncp:
      Serial.println(F("Can't move. Home position not set."));
      break;
    case fp_dis:
      Serial.println(F("Can't move. stepper motors disabled."));
      break;
    case fp_mov:
      Serial.println(F("Can't do this while moving."));
      break;
    case fp_nom:
      Serial.println(F("Can't do this unless moving."));
  }
}

/**
 * 
 * ui command handlers
 * 
 **/

// Unknown
void onUnknown() {
  Serial.println(F("Unknown or unimplemented command."));
}

// enable [0 | 1]
void onEnable() {
  bool en = ui.getWord(1).toInt() != 0;
  bool response = en ? diver.enableOutputs() : diver.disableOutputs();
  if (response == fp_ok) {
    Serial.print(F("Motor drivers "));
    Serial.println(en ? F("enabled.") : F("disabled."));
  } else {
    Serial.println(F("Can't disable motors while moving."));
  }
}

// help and h
void onHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  enable <1 | 0>           Enable or disable motor drivers."));
  Serial.println(F("  help                     Display this text."));
  Serial.println(F("  h                        Same as help."));
  Serial.println(F("  home                     Move to the home position."));
  Serial.println(F("  move <mmX> <mmY> <mmZ>   Move by the amounts specified"));
  Serial.println(F("  m <mmX> <mmY> <mmZ>      Same as move."));
  Serial.println(F("  setHome                  Assume the current position is home."));
  Serial.print(  F("                             (x: "));
  Serial.print(HOME_X);
  Serial.print(F("mm, y: "));
  Serial.print(HOME_Y);
  Serial.print(F("mm, z: "));
  Serial.print(HOME_Z);
  Serial.println(F("mm)"));
  Serial.println(F("  go                       Start the diver moving on its current heading."));
  Serial.println(F("  stop                     Stop all movement. Heading remains as is."));
  Serial.println(F("  s                        Same as stop command."));
  Serial.println(F("  dive <dir>               Dive up, level, or down. <dir> is \"u\", \"l\""));
  Serial.println(F("                             or \"d\"."));
  Serial.println(F("  turn <dir>               Turn diver in direction <dir> where <dir>"));
  Serial.println(F("                             is \"l\", \"s\" or \"r\"."));
  Serial.println(F("  where                    Print (z, y, z) location of diver."));
}

// m and move <x> <y> <z>
void onMove() {
  int8_t rc = diver.moveBy(fp_Point3D {
    mmToSteps(ui.getWord(1).toInt()), 
    mmToSteps(ui.getWord(2).toInt()), 
    mmToSteps(ui.getWord(3).toInt())});
  interpretRc(rc);
}

// go
void onGo() {
  interpretRc(diver.go());
}

// sethome
void onSethome(){
  interpretRc(diver.setCurrentPosition(fp_Point3D {mmToSteps(HOME_X), mmToSteps(HOME_Y), mmToSteps(HOME_Z)}));
  Serial.print(F("Home set to ("));
  Serial.print(HOME_X);
  Serial.print(F(", "));
  Serial.print(HOME_Y);
  Serial.print(F(", "));
  Serial.print(HOME_Z);
  Serial.println(F(")"));
}

// home
void onHome(){
  interpretRc(diver.moveTo(fp_Point3D {mmToSteps(HOME_X), mmToSteps(HOME_Y), mmToSteps(HOME_Z)}));
}

// s and stop
void onStop() {
  diver.stop();
  Serial.println(F("Stopping."));
}

// turn (l | r)
void onTurn() {
  String dirWord = ui.getWord(1);
  int8_t rc;
  if (dirWord.equals("l")) {
    rc = diver.turn(fp_left);
    Serial.println(F("Turning left"));
  }else if (dirWord.equals("r")) {
    Serial.println(F("Turning right."));
    rc = diver.turn(fp_right);
  } else {
    Serial.println(F("Ignored. Turn direction not \"l\" or \"r\"."));
    return;
  }
  interpretRc(rc);
  
}

// w and where
void onWhere() {
  fp_Point3D loc = diver.where();
  Serial.print(F("Diver is located at: ("));
  Serial.print(stepsToMm(loc.x));
  Serial.print(F(", "));
  Serial.print(stepsToMm(loc.y));
  Serial.print(F(", "));
  Serial.print(stepsToMm(loc.z));
  Serial.print(F(")  which is ("));
  Serial.print((stepsToMm(loc.x)) - HOME_X);
  Serial.print(F(", "));
  Serial.print(stepsToMm(loc.y) - HOME_Y);
  Serial.print(F(", "));
  Serial.print(stepsToMm(loc.z) - HOME_Z);
  Serial.println(F(") from home"));
}

/**
 * 
 * ControlPad state change handlers
 * 
 **/

// toForward
void onToForward() {
  #ifdef DEBUG
  Serial.print(F("cp: Forward. "));
  interpretRc(diver.go());
  #else
  diver.go();
  #endif
}

// toStop
void onToStop() {
  diver.stop();
  #ifdef DEBUG
  Serial.println(F("cp: Stop"));
  #endif
}

// toLeft
void onToLeft() {
  #ifdef DEBUG
  Serial.print(F("cp: Left. "));
  interpretRc(diver.turn(fp_left));
  #else
  diver.turn(fp_left);
  #endif
}

// toNeutral
void onToNeutral() {
  #ifdef DEBUG
  Serial.print(F("cp: Neutral. "));
  interpretRc(diver.turn(fp_straight));
  #else
  diver.turn(fp_straight);
  #endif
}

// toRight
void onToRight() {
  #ifdef DEBUG
  Serial.print(F("cp: Right. "));
  interpretRc(diver.turn(fp_right));
  #else
  diver.turn(fp_right);
  #endif
}

// downPressed
void onDownPressed() {
  #ifdef DEBUG
  Serial.print(F("cp: Falling. "));
  interpretRc(diver.turn(fp_falling));
  #else
  diver.turn(fp_falling);
  #endif
}

// downReleased or upReleased
void onUpOrDownReleased() {
  #ifdef DEBUG
  Serial.print("cp: Level. ");
  interpretRc(diver.turn(fp_level));
  #else
  diver.turn(fp_level);
  #endif
}

// upPressed
void onUpPressed() {
  #ifdef DEBUG
  Serial.print(F("cp: Rising. "));
  interpretRc(diver.turn(fp_rising));
  #else
  diver.turn(fp_rising);
  #endif
}

/**
 * 
 * Arduino setup() function. Called once at initialization
 * 
 **/
void setup() {
  Serial.begin(9600);
  Serial.println(BANNER);

  diver.begin();
  diver.setMaxSpeed(MAX_SPEED);
  diver.setSafetyMargins(
    mmToSteps(MARGIN_L), mmToSteps(MARGIN_R),
    mmToSteps(MARGIN_F), mmToSteps(MARGIN_B),
    mmToSteps(MARGIN_D), mmToSteps(MARGIN_U));

  // Attach the command handlers for the ui
  ui.attachDefaultCmdHandler(onUnknown);
  bool succeeded = 
  ui.attachCmdHandler("enable", onEnable) &&
  ui.attachCmdHandler("go", onGo) &&
  ui.attachCmdHandler("help", onHelp) &&
  ui.attachCmdHandler("h", onHelp) &&
  ui.attachCmdHandler("move", onMove) &&
  ui.attachCmdHandler("m", onMove) &&
  ui.attachCmdHandler("sethome", onSethome) &&
  ui.attachCmdHandler("home", onHome) &&
  ui.attachCmdHandler("stop", onStop) &&
  ui.attachCmdHandler("s", onStop) &&
  ui.attachCmdHandler("turn", onTurn) &&
  ui.attachCmdHandler("where", onWhere);
  if (!succeeded) {
    Serial.println(F("Too many command handlers."));
  }

  // Initialize control pad
  cp.begin();
  cp.attachHandler(toForward, onToForward);
  cp.attachHandler(toStop, onToStop);
  cp.attachHandler(toLeft, onToLeft);
  cp.attachHandler(toNeutral, onToNeutral);
  cp.attachHandler(toRight, onToRight);
  cp.attachHandler(upPressed, onUpPressed);
  cp.attachHandler(upReleased, onUpOrDownReleased);
  cp.attachHandler(downReleased, onUpOrDownReleased);
  cp.attachHandler(downPressed, onDownPressed);
}

/**
 * 
 * Arduino loop() function. Called repeatedly as soon as it returns.
 * 
 **/
void loop() {
  diver.run();    // Let the diver do its thing
  ui.run();       // Let the UI do its thing
  cp.run();       // Let the control pad do its
}