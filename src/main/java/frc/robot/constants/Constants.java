// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * Constants used throughout the code that are not categorized in other constants files.
 */
public final class Constants {
    /**
     * Tab names in Shuffleboard.
     */
    public static final class ShuffleboardTabNames {
        public static final String DEFAULT = "Competition";
        public static final String UTILITIES = "Utilities";
    }

    /** Constants for the controller and any controller related assignments. */
    public static final class ControllerConstants {
        /** DriverStation ID of the driver controller. */
        public static final int DRIVER_CONTROLLER_ID = 0;
        /** DriverStation ID of the operator controller. */
        public static final int OPERATOR_CONTROLLER_ID = 1;
        /** Removes input around the joystick's center (eliminates stick drift). */
        public static final double DEADBAND = 0.075;
        /** Whether or not to accept directional pad input for movement. */
        public static final boolean DPAD_DRIVE_INPUT = true;
        /** Speed multiplier when using fine-control movement. */
        public static final double FINE_CONTROL_MULT = 0.15;
    }

    /** Constants for the LED strips */
    public static final class LEDConstants {
        /** Id for PWM */
        public static final int PWM_HEADER = 0;
        /** i think this is how many leds nodes there are in a strip; not sure at all tho */
        public static final int LED_LENGTH = 70;
        /** Blink cooldown; time that the selected blink color will stay for, then blink */
        public static final double BLINK_COOLDOWN = 0.2;
    }
}
