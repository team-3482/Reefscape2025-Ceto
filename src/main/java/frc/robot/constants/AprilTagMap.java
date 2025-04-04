// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.Constants.AligningConstants;

/**
 * Every Reef Tag's position, hardcoded :)
 * Gotten from Limelight's fmap, turned the 4x4 matrixes back into 2d poses. 
 */
public final class AprilTagMap {
    private static final Map<Integer, Pose2d> REEF = Map.ofEntries(
        // Red side
        Map.entry( 6, new Pose2d(new Translation2d(13.474446, 3.306318), Rotation2d.fromDegrees(-60))),
        Map.entry( 7, new Pose2d(new Translation2d(13.890498, 4.0259  ), Rotation2d.fromDegrees(0))),
        Map.entry( 8, new Pose2d(new Translation2d(13.474446, 4.745482), Rotation2d.fromDegrees(60))),
        Map.entry( 9, new Pose2d(new Translation2d(12.643358, 4.745482), Rotation2d.fromDegrees(120))),
        Map.entry(10, new Pose2d(new Translation2d(12.227306, 4.0259  ), Rotation2d.fromDegrees(180))),
        Map.entry(11, new Pose2d(new Translation2d(12.643358, 3.306318), Rotation2d.fromDegrees(-120))),
        
        // Blue side
        Map.entry(17, new Pose2d(new Translation2d(4.073906, 3.306318), Rotation2d.fromDegrees(-120))),
        Map.entry(18, new Pose2d(new Translation2d(3.6576  , 4.0259  ), Rotation2d.fromDegrees(180))),
        Map.entry(19, new Pose2d(new Translation2d(4.073906, 4.745482), Rotation2d.fromDegrees(120))),
        Map.entry(20, new Pose2d(new Translation2d(4.90474 , 4.745482), Rotation2d.fromDegrees(60))),
        Map.entry(21, new Pose2d(new Translation2d(5.321046, 4.0259  ), Rotation2d.fromDegrees(0))),
        Map.entry(22, new Pose2d(new Translation2d(4.90474 , 3.306318), Rotation2d.fromDegrees(-60)))
    );

    private static final Transform2d FLIP = new Transform2d(Translation2d.kZero, Rotation2d.k180deg);

    /**
     * Gets a pose for a tag from an ID.
     * @param id - The ID to query.
     * @param flip - Whether to return the pose with the position +180.
     * @return The pose if the ID is valid, {@link Pose2d.kZero} if invalid.
     */
    public static Pose2d getPoseFromID(int id, boolean flip) {
        if (!AprilTagMap.REEF.containsKey(id)) {
            return Pose2d.kZero;
        }
        else if (flip) {
            return AprilTagMap.REEF.get(id).transformBy(AprilTagMap.FLIP);
        }
        else {
            return AprilTagMap.REEF.get(id);
        }
    }

    /**
     * Gets a pose for a tag from an ID.
     * @param id - The ID to query.
     * @return The pose if the ID is valid, {@link Pose2d.kZero} if invalid.
     */
    public static Pose2d getPoseFromID(int id) {
        return AprilTagMap.getPoseFromID(id, false);
    }

    /**
     * Calculates the position to align to for this tag.
     * @param id - The id of the tag.
     * @param direction - The direction to align, robot-relative left or right (-1, 0, 1).
     * @return The position. If the tag is not a REEF tag, returns {@link Pose2d.kZero}.
     */
    public static Pose2d calculateReefAlignedPosition(int id, int direction) {
        if (!AprilTagMap.REEF.containsKey(id)) return Pose2d.kZero;

        Pose2d tag = AprilTagMap.REEF.get(id);
        Rotation2d perpRot = tag.getRotation();
        
        double xChange = AligningConstants.Reef.PERPENDICULAR_DIST_TO_TAG * perpRot.getCos();
        double yChange = AligningConstants.Reef.PERPENDICULAR_DIST_TO_TAG * perpRot.getSin();
        
        if (direction != 0) {
            Rotation2d parlRot = perpRot.plus(Rotation2d.kCCW_Pi_2);

            xChange += Math.signum(direction) * AligningConstants.Reef.PARALLEL_DIST_TO_TAG * parlRot.getCos();
            yChange += Math.signum(direction) * AligningConstants.Reef.PARALLEL_DIST_TO_TAG * parlRot.getSin();
        }

        return new Pose2d(
            tag.getTranslation().plus(new Translation2d(xChange, yChange)),
            perpRot.plus(Rotation2d.fromDegrees(180))
        );
    }
}
