// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.wpilibj.util.Color;

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

    /** Colors used with the LEDSubsystem, these are named for readability. */
    public static final class NamedColors {
        public static final Color OFF = Color.kBlack;
        public static final Color ERROR = Color.kRed;
        public static final Color WARNING = Color.kOrange;
        public static final Color OK = Color.kGreen;
        
        public static final Color CORAL = Color.kWhite;
        public static final Color ALGAE = Color.kBlue;
    }

    public static final class ScoringConstants {
        /* Heights in elevator meters for scoring at these heights. */
        public static final double L1_HEIGHT = 0; // TODO FIELD/ELEVATOR : Test these with field elements
        public static final double L2_HEIGHT = 0;
        public static final double L3_HEIGHT = 0;
        public static final double BOTTOM_HEIGHT = 0;
    }
}
