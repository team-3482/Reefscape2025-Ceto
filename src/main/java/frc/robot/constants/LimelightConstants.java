// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Constants for Limelight-related code. */
public final class LimelightConstants {
    /** Enables publishing of the CameraFeeds to dashboard on startup. */
    public static final boolean PUBLISH_CAMERA_FEEDS = true;
    
    /** Bottom Limelight (Reef). */
    public static final String BOTTOM_RIGHT_LL = "limelight-stheno";
    /** Top Limelight (Processor/Barge). */
    public static final String BOTTOM_LEFT_LL = "limelight-euryale";

    /**
     * The distance within which to use Limelight data in meters. This is measured from tag to camera.
     * When entering this range, the VisionSubsystem will reset the odometry to the current LL pose.
     */
    public static final double REEF_TRUST_RANGE = 1.5;
    public static final double TRUST_RANGE = 5;
    public static final double REEF_ALIGN_RANGE = 2.0;

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
    public static final double[] DEFAULT_BOTTTOM_CROP = new double[]{ -1, 1, -1, 1 };
    
    /** Horizontal resolution of limelight in pixels. */
    public static final double RES_X = 1280;
    /** Vertical resolution of limelight in pixels. */
    public static final double RES_Y = 800;

    /** Increase any crop by this value around a pixel extremity. */
    public static final double BOUNDING_BOX = 0.3;
}
