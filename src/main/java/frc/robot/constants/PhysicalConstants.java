// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.util.Units;

/**
 * Constants used throughout the code specifically related to subsystems or unchangeable aspects of the bot.
 * @implSpec FRONT of the bot is where the elevator is, use that as a reference for directions.
 */
public final class PhysicalConstants {
    /**
     * Constants for physical attributes of the robot.
     */
    public static final class RobotConstants {
        /** Name of the CTRE CAN bus (configured on the CANivore). */
        public static final String CTRE_CAN_BUS = "ctre";
    }

    /** Constants for the LED strips */
    public static final class LEDConstants {
        /** Id for PWM */
        public static final int PWM_HEADER = 0;
        /** This is how many nodes are on the strip */
        public static final int LED_LENGTH = 72;
        /** Blink cooldown; time that the selected blink color will stay for, then blink */
        public static final double BLINK_COOLDOWN = 0.2;
    }

    /** Constants for the Elevator system */
    public static final class ElevatorConstants {
        /* Motor IDs */
        public static final int LEFT_MOTOR_ID = 20;
        public static final int RIGHT_MOTOR_ID = 21;

        /* Limit switch IDs on the DIO */
        public static final int BOTTOM_LASER_ID = 0;
        public static final int STAGE_TWO_UPPER_LASER_ID = 1;
        public static final int STAGE_THREE_UPPER_LASER_ID = 2;

        /** 
         * The dimater of the bar that the string wraps around on the elevator
         * @implSpec This is in meters and is very important to calculate the position of the elevator
         */
        public static final double ROLLER_DIAMETER = Units.inchesToMeters(1.25);
        /**
         * Heuristic constant found because the math was off by this coefficient.
         */
        public static final double LINEAR_CONSTANT_MULT = 0.75;

        /**
         * Tolerance used for elevator command in meters.
         * @see {@link MoveElevatorCommand}
         */
        public static final double HEIGHT_TOLERANCE = 0.05;

        /* Motion Magic Config */
        public static final double ROTOR_TO_MECHANISM_RATIO = (double) 36 / 18;
        public static final double CRUISE_SPEED = 30;
        public static final double SLOW_CRUISE_SPEED = 10;
        public static final double ACCELERATION = 60;
        public static final double SLOW_ACCELERATION = 30;

        /** Gains used for MotionMagic slot 0. */
        public static final class ElevatorSlot0Gains {
            public static final double kG = 0.35;
            public static final double kS = 0.1;
            public static final double kV = 0;
            public static final double kA = 0;
            public static final double kP = 28;
            public static final double kI = 0;
            public static final double kD = 0;
        }
    }

    public static final class AlgaeConstants {
        public static final int RIGHT_MOTOR_ID = 41;
        public static final int LEFT_MOTOR_ID = 40;

        public static final double INTAKE_VOLTAGE = 5;
        public static final double OUTTAKE_VOLTAGE = 3;
        public static final double HOLDING_VOLTAGE = 0.45;

        public static final double MECHANISM_RATIO = 1;
    }

    public static final class CoralConstants {
        public static final int RIGHT_MOTOR_ID = 31;
        public static final int LEFT_MOTOR_ID = 30;

        public static final int FRONT_LASER_ID = 3;
        public static final int BACK_LASER_ID = 4;

        public static final double INTAKE_VOLTAGE = 2;
        public static final double SLOW_INTAKE_VOLTAGE = 0.75;
        public static final double OUTTAKE_VOLTAGE = 2;
        
        public static final double MECHANISM_RATIO = 1;
    }
}
