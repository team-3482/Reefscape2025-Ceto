// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * Constants used throughout the code specifically related to subsystems or unchangeable aspects of the bot.
 * @implSpec BACK of the bot is 180 degrees / the battery, use that as a reference for directions.
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
        public static final int LED_LENGTH = 70;
        /** Blink cooldown; time that the selected blink color will stay for, then blink */
        public static final double BLINK_COOLDOWN = 0.2;
    }

    /** Constants for the Elevator system */
    public static final class ElevatorConstants {
        /** Motion Magic Config */
        public static final double ROTOR_TO_MECHANISM_RATIO = (double) 1 / 25;
        public static final double CRUISE_SPEED = 0;
        public static final double ACCELERATION = 0;

        /**
         * Tolerance used for elevator command in meters.
         * @see {@link ElevatorCommand}
         */
        public static final double HEIGHT_TOLERANCE = 0.01;

        /* Motor IDs */
        public static final int LEFT_MOTOR_ID = 0;
        public static final int RIGHT_MOTOR_ID = 0;

        /** Physical Constants (Hard stops) */
        public static final double LOWER_STOP = 0;
        public static final double UPPER_STOP = 0;

        /* Limit switch IDs on the DIO */
        public static final int BOTTOM_LIMIT_ID = 0;
        public static final int UPPER_LIMIT_ID = 0;

        /** 
         * The dimater of the bar that rolls to control the elevator
         * @implSpec this is in meters and is very important to calculate the position of the elevator
         */
        public static final double ROLLER_DIAMETER = 0;

        /** Gains used for MotionMagic slot 0. */
        public static final class ElevatorSlot0Gains {
            public static final double kG = 0;
            public static final double kS = 0;
            public static final double kV = 0;
            public static final double kA = 0;
            public static final double kP = 0;
            public static final double kI = 0;
            public static final double kD = 0;
        }
    }
}
