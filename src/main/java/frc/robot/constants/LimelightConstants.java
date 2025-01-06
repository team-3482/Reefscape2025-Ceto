// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import frc.robot.limelights.VisionSubsystem;

/** Constants for Limelight-related code. */
public final class LimelightConstants {
    /** Enables publishing of the CameraFeeds to Shuffleboard on startup. */
    public static final boolean PUBLISH_CAMERA_FEEDS = true;
    
    /** Front left Limelight (April Tags). */
    public static final String LEFT_LL = "limelight-stheno";
    /** Front right Limelight (April Tags). */
    public static final String RIGHT_LL = "limelight-euryale";

    /** The distance within which to use Limelight data in meters. This is measured from tag to camera.*/
    public static final int TRUST_TAG_DISTANCE = 10;

    /** All valid tag IDs (used for tag filtering) */
    public static final int[] ALL_TAG_IDS = new int[]{
        1, 2, 3, 4, 5,
        6, 7, 8, 9, 10,
        11, 12, 13, 14, 15,
        16, 17, 18, 19, 20,
        21, 22
    };
    
    /** Crop window size when no tags are in view (used for smart cropping) */
    public static final double DEFAULT_CROP_SIZE = 0.85;

    /**
     * The time limit for considering data to be recent in seconds.
     * @see {@link VisionSubsystem#recentVisionData()}
     */
    public static final double RECENT_DATA_CUTOFF = 3.5;
    
    // See https://docs.limelightvision.io/docs/docs-limelight/hardware-comparison.
    /** Horizontal FOV of LL3G in degrees (used for smart cropping) */
    public static final double FOV_X = 82;
    /** Vertical FOV of LL3G in degrees (used for smart cropping) */
    public static final double FOV_Y = 56.2;
    /** FOV area of the LL3g in degrees squared (used for smart cropping) */
    public static final double FOV_AREA = FOV_X * FOV_Y;
}