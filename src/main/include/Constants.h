// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

#include <units/time.h>
#include <units/length.h>


const int MOTOR_LEFT = 0;
const int MOTOR_RIGHT = 1;

const int DRIVER_JOYSTICK = 0;

const int BUTTON_A = 1;
const int BUTTON_B = 2;
const int BUTTON_X = 3;
const int BUTTON_Y = 4;

const int LEFTSTICK_X = 0;
const int LEFTSTICK_Y = 1;
const int RIGHTSTICK_X = 4;
const int RIGHTSTICK_Y = 5;

const int LEFT_TRIGGER = 2;
const int RIGHT_TRIGGER = 3;

//Drivetrain variables
const double TICKS_PER_REVOLUTION = 1440.0;
const units::meter_t WHEEL_DIAMETER = 70_mm;

const double DRIVE_SPEED = 0.5;

const units::meter_t MIN_STOP_SPEED = 2_cm;

//Teleop modes
const double TURN_SPEED_MODIFIER = 0.5;
const enum class CONTROL_SCHEME {TRIGGERS, ONE_STICK, TWO_STICK, TANKDRIVE, CHAOS, QWOP};
const units::second_t CHAOS_INTERVAL = 10_s;
const CONTROL_SCHEME CHAOS_POSSIBILITIES[] = {CONTROL_SCHEME::TRIGGERS, CONTROL_SCHEME::ONE_STICK, CONTROL_SCHEME::TWO_STICK, CONTROL_SCHEME::TANKDRIVE, CONTROL_SCHEME::QWOP};
const std::string CHAOS_MESSAGES[] = {"CHAOS CHAOS CHAOS", "The controls shift...", "Something has changed.", "Something's wrong, I can feel it."}; 

