package frc.robot.limelights;

import frc.robot.constants.LimelightConstants;

/**
 * <p>A helper class used for storing MegaTag and MegaTag2 data from a Limelight
 * to avoid unnecessary calls to the NetworkTables API.
 * @apiNote MT rotation should not be combined with MT2 pose, because their timestamps may differ.
 */
public class VisionData {
    public final String name;
    public final LimelightHelpers.PoseEstimate MegaTag;
    public final LimelightHelpers.PoseEstimate MegaTag2;
    public final boolean canTrustRotation;
    public final boolean canTrustPosition;
    /**
     * Flag set after optimization to avoid re-optimizing data twice in a row on low FPS.
     * This also avoids using the data twice, because optimized data has been processed.
     */
    public boolean optimized;

    /**
     * Creates a new LimelightData object.
     * @param name - The name of this Limelight.
     * @param MegaTag data.
     * @param MegaTag2 data.
     */
    public VisionData(String name, LimelightHelpers.PoseEstimate MegaTag, LimelightHelpers.PoseEstimate MegaTag2) {
        this.name = name;
        this.MegaTag = MegaTag;
        this.MegaTag2 = MegaTag2;
        this.optimized = false;

        // These are calculated when the measurement is created
        this.canTrustRotation = canTrustRotation();
        this.canTrustPosition = canTrustPosition();
    }

    /**
     * Checks if the average tag distance is within 3 meters and MegaTag has >= 2 targets.
     * @return Whether rotation data can be trusted.
     */
    private boolean canTrustRotation() {
        return this.MegaTag2 != null
            && this.MegaTag2.avgTagDist <= 3 // 3 Meters
            && this.MegaTag != null
            && this.MegaTag.tagCount >= 2;
    }

    /**
     * Checks if the average tag distance is within {@link LimelightConstants#TRUST_TAG_DISTANCE} of the robot.
     * @return Whether position data can be trusted.
     */
    private boolean canTrustPosition() {
        return this.MegaTag2 != null
            && this.MegaTag2.tagCount > 0
            && this.MegaTag2.avgTagDist < LimelightConstants.TRUST_TAG_DISTANCE;
    }
}
