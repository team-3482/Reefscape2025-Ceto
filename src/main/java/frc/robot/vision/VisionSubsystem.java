// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import java.util.Iterator;
import java.util.Map;
import java.util.Optional;

import com.ctre.phoenix6.Utils;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ShuffleboardTabNames;
import frc.robot.constants.Constants.TagSets;
import frc.robot.led.LEDSubsystem;
import frc.robot.led.StatusColors;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.constants.LimelightConstants;

import org.littletonrobotics.junction.Logger;

/** 
 * A class that manages AprilTag Limelights for vision.
 * <p>Optimizes detection for better performance and pushes
 * position updates to the internal odometer.
 */
public class VisionSubsystem extends SubsystemBase {
    // Use Bill Pugh Singleton Pattern for efficient lazy initialization (thread-safe !)
    private static class VisionSubsystemHolder {
        private static final VisionSubsystem INSTANCE = new VisionSubsystem();
    }
    
    public static VisionSubsystem getInstance() {
        return VisionSubsystemHolder.INSTANCE;
    }

    /** Used to run vision processing on a separate thread. */
    private final Notifier notifier;

    /**
     * Latest Limelight data. May contain faulty data unsuitable for odometry.
     * @apiNote [ Left, Right ]
     */
    private VisionData[] limelightDatas = new VisionData[2];
    /** Last heartbeat of the front LL (updated every frame) */
    private long lastHeartbeatBottomRightLL = 0;
    /** Last heartbeat of the back LL (updated every frame) */
    private long lastHeartbeatBottomLeftLL = 0;
    /**
     * Timer used to track when the cameras last got data.
     * @apiNote It would probably be better to track distance traveled instead,
     * but this was the quickest solution.
     */
    private final Timer lastDataTimer = new Timer();

    /* Shuffleboard */
    private final ShuffleboardLayout shuffleboardLayout = Shuffleboard.getTab(ShuffleboardTabNames.DEFAULT)
        .getLayout("VisionSubsystem", BuiltInLayouts.kGrid)
        .withProperties(Map.of("Number of columns", 1, "Number of rows", 2, "Label position", "TOP"))
        .withSize(5, 3)
        .withPosition(1, 2);
    private GenericEntry shuffleboardProcessorInView = shuffleboardLayout
        .add("Processor Align", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of(
            "colorWhenTrue", StatusColors.CAN_ALIGN.color.toHexString(),
            "colorWhenFalse", StatusColors.OFF.color.toHexString()
        ))
        .withSize(3, 1)
        .withPosition(0, 1)
        .getEntry();
    private GenericEntry shuffleboardReefInView = shuffleboardLayout
        .add("Reef Align", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of(
            "colorWhenTrue", StatusColors.CAN_ALIGN.color.toHexString(),
            "colorWhenFalse", StatusColors.OFF.color.toHexString()
        ))
        .withSize(3, 1)
        .withPosition(0, 2)
        .getEntry();
    
    private boolean lastReef = false;
    private boolean lastProcessor = false;
    private boolean lastCanAlign = false;

    /** Creates a new VisionSubsystem. */
    private VisionSubsystem() {
        super("VisionSubsystem");

        if (LimelightConstants.PUBLISH_CAMERA_FEEDS) {
            // Shuffleboard camera feeds.
            HttpCamera bottomRightLLCamera = new HttpCamera(
                LimelightConstants.BOTTOM_RIGHT_LL,
                "http://" + "10.34.82.12" + ":5800/stream.mjpg"
            );
            HttpCamera bottomLeftLLCamera = new HttpCamera(
                LimelightConstants.BOTTOM_LEFT_LL,
                "http://" + "10.34.82.13" + ":5800/stream.mjpg"
            );

            Shuffleboard.getTab(ShuffleboardTabNames.DEFAULT)
                .add(LimelightConstants.BOTTOM_LEFT_LL, bottomLeftLLCamera)
                .withWidget(BuiltInWidgets.kCameraStream)
                .withProperties(Map.of("Show Crosshair", false, "Show Controls", false))
                .withSize(7, 4)
                .withPosition(6, 0);
            Shuffleboard.getTab(ShuffleboardTabNames.DEFAULT)
                .add(LimelightConstants.BOTTOM_RIGHT_LL, bottomRightLLCamera)
                .withWidget(BuiltInWidgets.kCameraStream)
                .withProperties(Map.of("Show Crosshair", false, "Show Controls", false))
                .withSize(7, 4)
                .withPosition(6, 4);
        }

        LimelightHelpers.SetFiducialIDFiltersOverride(LimelightConstants.BOTTOM_RIGHT_LL, LimelightConstants.ALL_TAG_IDS);
        LimelightHelpers.SetFiducialIDFiltersOverride(LimelightConstants.BOTTOM_LEFT_LL, LimelightConstants.ALL_TAG_IDS);

        this.lastDataTimer.start();

        this.notifier = new Notifier(this::notifierLoop);
        this.notifier.setName("Vision Notifier");
        // The processing takes no longer than a regular robot cycle.
        // FPS will never be high enough to take advantage of every cycle,
        // but it's fine because repeat frames are entirely ignored (see heartbeats).
        this.notifier.startPeriodic(0.02);
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        // Uses a Notifier for separate-thread Vision processing
        // These methods are here because they are NOT thread-safe
        boolean processor, reef, canAlign;
        processor = reef = canAlign = false;

        if (recentVisionData()) {
            int primaryTag = getPrimaryTagInView();
            
            reef = TagSets.REEF_TAGS.contains(primaryTag);
            processor = TagSets.PROCESSOR_TAGS.contains(primaryTag);
            canAlign = reef || processor;
        }
        
        if (reef != this.lastReef) {
            this.shuffleboardReefInView.setBoolean(reef);
            Logger.recordOutput("Vision/ReefInView", reef);
            this.lastReef = reef;
        }
        if (processor != this.lastProcessor) {
            this.shuffleboardProcessorInView.setBoolean(processor);
            Logger.recordOutput("Vision/ProcessorInView", processor);
            this.lastProcessor = processor;
        }
        if (canAlign != this.lastCanAlign) {
            Logger.recordOutput("Vision/canAlign", processor || reef);
            this.lastCanAlign = canAlign;
        }
        
        if (canAlign && DriverStation.isEnabled()) {
            LEDSubsystem.getInstance().setColor(StatusColors.CAN_ALIGN);
        }
    }

    /**
     * This method is used in conjunction with a Notifier to run vision processing on a separate thread.
     */
    private void notifierLoop() {
        // This loop generally updates data in about 6 ms, but may double or triple for no apparent reason.
        // This causes loop overrun warnings, however, it doesn't seem to be due to inefficient code and thus can be ignored.
        for (VisionData data : fetchLimelightData()) { // This method gets data in about 6 to 10 ms.
            if (data.optimized || data.MegaTag == null || data.MegaTag2 == null) continue;

            if (data.canTrustRotation()) {
                // Only trust rotational data when adding this pose.
                SwerveSubsystem.getInstance().setVisionMeasurementStdDevs(VecBuilder.fill(
                    9999999,
                    9999999,
                    recentVisionData() ? 1 : 0.4
                ));
                SwerveSubsystem.getInstance().addVisionMeasurement(
                    data.MegaTag.pose,
                    Utils.fpgaToCurrentTime(data.MegaTag.timestampSeconds)
                );
            }

            if (data.canTrustPosition()) {
                // Only trust positional data when adding this pose.
                SwerveSubsystem.getInstance().setVisionMeasurementStdDevs(VecBuilder.fill(
                    recentVisionData() ? 0.7 : 0.1,
                    recentVisionData() ? 0.7 : 0.1,
                    9999999
                ));
                SwerveSubsystem.getInstance().addVisionMeasurement(
                    data.MegaTag2.pose,
                    Utils.fpgaToCurrentTime(data.MegaTag2.timestampSeconds)
                );
            }

            hasRecentVisionData(
                data.canTrustPosition() ? data.MegaTag2.pose.getTranslation() : null,
                data.canTrustRotation() ? data.MegaTag.pose.getRotation() : null
            );
        }
        // This method is suprprisingly efficient, generally below 1 ms.
        optimizeLimelights();
    }

    /**
     * Updates and returns {@link VisionSubsystem#limelightDatas}.
     * @return LimelightData for each trustworthy Limelight data.
     * @apiNote Will theoretically stop updating data if the heartbeat resets.
     * However, this happens at 2e9 frames, which would take consecutive 96 days at a consistent 240 fps.
     */
    private VisionData[] fetchLimelightData() {
        long heartbeatBottomRightLL = -1;
        long heartbeatBottomLeftLL = -1;

        // Periodic logic
        double rotationDegrees = SwerveSubsystem.getInstance().getState().Pose.getRotation().getDegrees();
        LimelightHelpers.SetRobotOrientation(LimelightConstants.BOTTOM_RIGHT_LL,
            rotationDegrees, 0, 0, 0, 0, 0
        );
        LimelightHelpers.SetRobotOrientation(LimelightConstants.BOTTOM_LEFT_LL,
            rotationDegrees, 0, 0, 0, 0, 0
        );
        
        heartbeatBottomRightLL = LimelightHelpers.getLimelightNTTableEntry(LimelightConstants.BOTTOM_RIGHT_LL, "hb").getInteger(-1);
        heartbeatBottomLeftLL = LimelightHelpers.getLimelightNTTableEntry(LimelightConstants.BOTTOM_LEFT_LL, "hb").getInteger(-1);

        if (heartbeatBottomLeftLL == -1 || this.lastHeartbeatBottomLeftLL < heartbeatBottomLeftLL) {
            this.limelightDatas[0] = getVisionData(LimelightConstants.BOTTOM_LEFT_LL);
            this.lastHeartbeatBottomLeftLL = heartbeatBottomLeftLL == -1 ? this.lastHeartbeatBottomLeftLL : heartbeatBottomLeftLL;
        }

        if (heartbeatBottomRightLL == -1 || this.lastHeartbeatBottomRightLL < heartbeatBottomRightLL) {
            this.limelightDatas[1] = getVisionData(LimelightConstants.BOTTOM_RIGHT_LL);
            this.lastHeartbeatBottomRightLL = heartbeatBottomRightLL == -1 ? this.lastHeartbeatBottomRightLL : heartbeatBottomRightLL;
        }

        // There is no point actually filtering out nonexistent or null data,
        // because the code in the notifierLoop method will call VisionData's
        // methods to see if the data is valid for position/rotation.
        return this.limelightDatas;
    }

    private VisionData getVisionData(String limelight) {
        LimelightHelpers.PoseEstimate mt = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight);

        double leftX = -1;
        double rightX = -1;
        double bottomY = -1;
        double topY = -1;

        // The JSON processing takes 2-4 ms generally
        // This has to exist because "rawdetections" is empty in NetworkTables
        // If Limelight ever fixes that, then this code can be replaced entirely
        // and use the corners from rawdetections.
        try {
            ObjectMapper mapper = new ObjectMapper();
            JsonNode node = mapper.readTree(LimelightHelpers.getJSONDump(limelight));
            Iterator<JsonNode> fiducials = node.get("Fiducial").elements();
            
            while (fiducials.hasNext()) {
                JsonNode pts = fiducials.next().get("pts");
                for (int i = 0; i < 4; i++) {
                    JsonNode xy = pts.get(i);
                    if (xy.get(0).asDouble(-1) < leftX || leftX == -1) {
                        leftX = xy.get(0).asDouble(-1);
                    }
                    if (xy.get(0).asDouble(-1) > rightX || rightX == -1) {
                        rightX = xy.get(0).asDouble(-1);
                    }
                    // Y-origin is at the top for some reason
                    if (800 - xy.get(1).asDouble(-1) < bottomY || bottomY == -1) {
                        bottomY = 800 - xy.get(1).asDouble(-1);
                    }
                    if (800 - xy.get(1).asDouble(-1) > topY || topY == -1) {
                        topY = 800 - xy.get(1).asDouble(-1);
                    }
                }
            }
        }
        // JsonProcessingException is thrown by readTree()
        // NullPointerException is thrown when the LL hasn't yet populated the "json" NT entry
        // This catches all errors because if this breaks it will crash robot code
        // This fallback uses only the primary target's corners instead of all target corners
        catch (Exception e) {
            DoubleArrayEntry cornersEntry = LimelightHelpers.getLimelightDoubleArrayEntry(limelight, "tcornxy");
            double[] corners = cornersEntry.get(new double[0]);

            for (int i = 0; i < corners.length; i++) {
                if (corners[i] < leftX || leftX == -1) {
                    leftX = corners[i];
                }
                if (corners[i] > rightX || rightX == -1) {
                    rightX = corners[i];
                }
                i++;
                if (800 - corners[i] < bottomY || bottomY == -1) {
                    bottomY = 800 - corners[i];
                }
                if (800 - corners[i] > topY || topY == -1) {
                    topY = 800 - corners[i];
                }
            }
        }

        return new VisionData(limelight, mt, mt2, leftX, rightX, bottomY, topY);
    }

    /**
     * A helper method used to optimize Limelight FPS.
     */
    private void optimizeLimelights() {
        for (VisionData limelightData : this.limelightDatas) {
            if (limelightData == null || limelightData.optimized) {
                continue;
            }
            else {
                limelightData.optimized = true;
            }
            
            // Avoid unnecessary optimization for a LL with no tags and
            // reset any optimization that might have been done previously.
            if (limelightData.MegaTag2 == null || limelightData.MegaTag2.tagCount == 0) {
                LimelightHelpers.SetFiducialDownscalingOverride(limelightData.name, 1.0f);
                LimelightHelpers.setCropWindow(
                    limelightData.name,
                    LimelightConstants.DEFAULT_BOTTTOM_CROP[0],
                    LimelightConstants.DEFAULT_BOTTTOM_CROP[1],
                    LimelightConstants.DEFAULT_BOTTTOM_CROP[2],
                    LimelightConstants.DEFAULT_BOTTTOM_CROP[3]
                );
                continue;
            }

            // Downscaling closer to tags.
            if (DriverStation.isDisabled()) {
                LimelightHelpers.SetFiducialDownscalingOverride(limelightData.name, 1.0f);
            }
            else if (limelightData.MegaTag2.avgTagDist < 1) {
                LimelightHelpers.SetFiducialDownscalingOverride(limelightData.name, 3.0f);
            }
            else if (limelightData.MegaTag2.avgTagDist < 2) {
                LimelightHelpers.SetFiducialDownscalingOverride(limelightData.name, 2.0f);
            }
            else if (limelightData.MegaTag2.avgTagDist < 3) {
                LimelightHelpers.SetFiducialDownscalingOverride(limelightData.name, 1.5f);
            }
            else {
                LimelightHelpers.SetFiducialDownscalingOverride(limelightData.name, 1.0f);
            }

            // Smart cropping around on-screen AprilTags
            if (limelightData.leftX == -1) {
                LimelightHelpers.setCropWindow(
                    limelightData.name,
                    LimelightConstants.DEFAULT_BOTTTOM_CROP[0],
                    LimelightConstants.DEFAULT_BOTTTOM_CROP[1],
                    LimelightConstants.DEFAULT_BOTTTOM_CROP[2],
                    LimelightConstants.DEFAULT_BOTTTOM_CROP[3]
                );
            }
            else {
                double leftCrop = limelightData.leftX / (LimelightConstants.RES_X / 2) - 1;
                double rightCrop = limelightData.rightX / (LimelightConstants.RES_X / 2) - 1;
                double bottomCrop = limelightData.bottomY / (LimelightConstants.RES_Y / 2) - 1;
                double topCrop = limelightData.topY / (LimelightConstants.RES_Y / 2) - 1;
                
                // Commented out because we will not be seeing more than one tag at once with the bottom LLs this season
                // if (
                //     limelightData.MegaTag2.tagCount == 1
                //     && (17 <= limelightData.MegaTag2.rawFiducials[0].id
                //         || (6 <= limelightData.MegaTag2.rawFiducials[0].id
                //             && limelightData.MegaTag2.rawFiducials[0].id <= 11))
                // ) {
                //     leftCrop -= 1.4 / limelightData.MegaTag2.avgTagDist;
                //     rightCrop += 1.4 / limelightData.MegaTag2.avgTagDist;
                // }
                // else {
                leftCrop -= LimelightConstants.BOUNDING_BOX;
                rightCrop += LimelightConstants.BOUNDING_BOX;

                // }

                if (DriverStation.isAutonomous()) {
                    leftCrop = -1;
                    rightCrop = 1;
                }
                
                bottomCrop -= LimelightConstants.BOUNDING_BOX;
                topCrop += LimelightConstants.BOUNDING_BOX;

                LimelightHelpers.setCropWindow(limelightData.name, leftCrop, rightCrop, bottomCrop, topCrop);
            }
        }
    }

    /**
     * Returns if vision data has been processed in the last {@link LimelightConstants#RECENT_DATA_CUTOFF}.
     * @return If there is recent vision data.
     */
    public boolean recentVisionData() {
        return !this.lastDataTimer.hasElapsed(LimelightConstants.RECENT_DATA_CUTOFF);
    }

    /**
     * A method that restarts the timer if there is recent data.
     * @param translation - The estimated translation.
     * @param rotation - The estimated rotation.
     * @apiNote If either is {@code null}, it will be ignored.
     */
    private void hasRecentVisionData(Translation2d translation, Rotation2d rotation) {
        Pose2d odometryPose = SwerveSubsystem.getInstance().getState().Pose;

        boolean validTranslation = translation == null ? false
            : odometryPose.getTranslation().getDistance(translation) <= 0.3;
        
        boolean validRotation = rotation != null;
        if (validRotation) {
            double oAngle = odometryPose.getRotation().getDegrees();
            double lAngle = rotation.getDegrees();
            double minAngle = Math.min(360 - Math.abs(oAngle - lAngle), Math.abs(oAngle - lAngle));
            validRotation = minAngle < 10;
        }

        if (validTranslation && validRotation) {
            this.lastDataTimer.restart();
        }
    }

    /**
     * Gets the robot position according to the Limelight in target-space (the target is at the origin).
     * @return The robot position.
     * @apiNote This method will grab data from whichever Limelight sees a tag, with priority for the bottom right one.
     * Returns an empty optional if there is no tags in view for either Limelight.
     */
    public Optional<Pose2d> getEstimatedPosition_TargetSpace() {
        return getEstimatedPosition_TargetSpace(LimelightConstants.BOTTOM_RIGHT_LL)
            .or(() -> getEstimatedPosition_TargetSpace(LimelightConstants.BOTTOM_LEFT_LL));
    }

    /**
     * Gets the robot position according to the Limelight in target-space (the target is at the origin).
     * @param limelight - The limelight to source the position from.
     * @return The robot position.
     * Returns an empty optional if there is no tags in view for the limelight.
     */
    private Optional<Pose2d> getEstimatedPosition_TargetSpace(String limelight) {
        /* [ x, z, y, pitch, yaw, roll ] (meters, degrees) */
        double[] poseArray = LimelightHelpers.getBotPose_TargetSpace(limelight);

        // 0, 0, 0
        if (poseArray[0] == 0 && poseArray[2] == 0 && poseArray[4] == 0) {
            return Optional.empty();
        }

        return Optional.of(new Pose2d(
            new Translation2d(poseArray[0], poseArray[2]),
                Rotation2d.fromDegrees(poseArray[4])
        ));
    }

    /**
     * Gets the primary tag in view of either bottom limelights, with priority for the right one.
     * @return The tag ID.
     */
    public int getPrimaryTagInView() {
        int id = getPrimaryTagInView(LimelightConstants.BOTTOM_RIGHT_LL);
        if (id == -1) {
            id = getPrimaryTagInView(LimelightConstants.BOTTOM_LEFT_LL);
        }
        return id;
    }

    /**
     * Gets the primary tag in view of the limelight.
     * @param limelight - The limelight to get the ID for.
     * @return The tag ID.
     */
    private int getPrimaryTagInView(String limelight) {
        return (int) NetworkTableInstance.getDefault().getTable(limelight)
            .getEntry("tid").getInteger(-1);
    }
}

// BOTTOM LEFT LL : Tune to robot position better :(