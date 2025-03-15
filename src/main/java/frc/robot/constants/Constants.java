// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import java.util.Set;

/** Constants used throughout the code that are not categorized in other constants files. */
public final class Constants {
    /** Tab names in Shuffleboard. */
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
  
    /** Constants used for auto-aligning the robot when scoring. */
    public static final class AligningConstants {
        public static final class Reef {
            /** How far (in meters) the robot should be from the tag perpendicularly to score. */
            public static final double PERPENDICULAR_DIST_TO_TAG_CORAL = 0.48;
            /** How far (in meters) the robot should be from the tag perpendicularly to score. */
            public static final double PERPENDICULAR_DIST_TO_TAG_ALGAE = 0.42;
            /** How far (in meters) the robot should be parallel to the tag to score. */
            public static final double PARALLEL_DIST_TO_TAG = 0.185;
        }

        public static final class Processor {
            /** How far (in meters) the robot should be from the tag perpendicularly to score. */
            public static final double PERPENDICULAR_DIST_TO_TAG = 0.7;
            /** How far (in meters) the robot should be parallel to the tag to score. */
            public static final double PARALLEL_DIST_TO_TAG = 0.0;
        }
    }

    /** Constants for elevator heights */
    public static final class ScoringConstants {
        /* Heights in elevator meters for scoring at these heights. */
        public static final double L1_CORAL = 0.235;
        public static final double L2_CORAL = 0.34;
        public static final double L2_ALGAE = 0.63;
        public static final double L3_CORAL = 0.74;
        public static final double BOTTOM_HEIGHT = 0; // Used for intaking also
        public static final double MAX_HEIGHT = 0.75;

    }
    
    /** Sets of tags for O(1) lookup. */
    public static final class TagSets {
        public static final Set<Integer> REEF_TAGS = Set.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);
        public static final Set<Integer> PROCESSOR_TAGS = Set.of(3, 16);
        public static final Set<Integer> BARGE_TAGS = Set.of(4, 5, 14, 15);

    }

    /** States used with Algae and Coral subsystems, reduces overhead. */
    public static enum SubsystemStates {
        STOPPED, INTAKING, SLOW_INTAKING, HOLDING, OUTTAKING, SLOW_OUTTAKING
    }
}
