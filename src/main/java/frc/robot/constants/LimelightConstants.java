// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import frc.robot.vision.VisionSubsystem;

/** Constants for Limelight-related code. */
public final class LimelightConstants {
    /** Enables publishing of the CameraFeeds to Shuffleboard on startup. */
    public static final boolean PUBLISH_CAMERA_FEEDS = true;
    
    /** Bottom Limelight (Reef). */
    public static final String BOTTOM_LL = "limelight-stheno";
    /** Top Limelight (Processor/Barge). */
    public static final String TOP_LL = "limelight-euryale";

    /** The distance within which to use Limelight data in meters. This is measured from tag to camera.*/
    public static final int TRUST_TAG_DISTANCE = 10;

    /**
     * The time limit for considering data to be recent in seconds.
     * @see {@link VisionSubsystem#recentVisionData()}
     */
    public static final double RECENT_DATA_CUTOFF = 3.0;

    /** All valid tag IDs (used for tag filtering) */
    public static final int[] ALL_TAG_IDS = new int[]{
        1, 2, 3, 4, 5,
        6, 7, 8, 9, 10,
        11, 12, 13, 14,
        15, 16, 17, 18,
        19, 20, 21, 22
    };
    
    /**
     * Crop window size when no tags are in view (used for smart cropping).
     * {@code xMin, xMax, yMin, yMax}.
     */
    // TODO TOP LL : When on final mount, configure default crops to be optimal
    public static final double[] DEFAULT_TOP_CROP = new double[]{ -0.75, 0.75, -0.85, 0.85 }; // Doubles FPS from 4 to 10 in busy scenes.
    public static final double[] DEFAULT_BOTTTOM_CROP = new double[]{ -0.85, 0.85, -0.7, 0.8 };
    
    /** Horizontal resolution of limelight in pixels. */
    public static final double RES_X = 1280;
    /** Vertical resolution of limelight in pixels. */
    public static final double RES_Y = 800;

    /** Increase any crop by this value around a pixel extremity. */
    public static final double BOUNDING_BOX = 0.2;
}