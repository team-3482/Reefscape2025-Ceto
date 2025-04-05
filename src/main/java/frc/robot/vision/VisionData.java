package frc.robot.vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

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

    // Lazy-loaded flags
    private Boolean canTrustRotation;
    private Boolean canTrustPosition;

    // Used for smart cropping optimization
    public final double leftX;
    public final double rightX;
    public final double bottomY;
    public final double topY;

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
     * @param leftX - Leftmost corner pixel x-coordinate.
     * @param rightY - Rightmost corner pixel x-coordinate.
     * @param bottomY - Bottommost corner pixel y-coordinate.
     * @param topY - Topmost corner pixel y-coordinate.
     */
    public VisionData(
        String name, LimelightHelpers.PoseEstimate MegaTag, LimelightHelpers.PoseEstimate MegaTag2,
        double leftX, double rightX, double bottomY, double topY
    ) {
        this.name = name;
        this.MegaTag = MegaTag;
        this.MegaTag2 = MegaTag2;
        this.optimized = false;

        this.leftX = leftX;
        this.rightX = rightX;
        this.bottomY = bottomY;
        this.topY = topY;
    }

    /**
     * Checks if the average tag distance is within 3 meters and MegaTag has >= 2 targets.
     * @return Whether rotation data can be trusted.
     */
    public boolean canTrustRotation() {
        if (this.canTrustRotation == null) {
            this.canTrustRotation = 
                this.MegaTag != null && this.MegaTag2 != null
                && (this.MegaTag.tagCount > 0
                    && (
                        this.MegaTag2.avgTagDist <= LimelightConstants.TRUST_RANGE
                        // Trust any data when disabled, so that it can set initial position / rotation.
                        || DriverStation.isDisabled()
                    )
                );
        }
        return this.canTrustRotation;
    }

    /**
     * Checks if the average tag distance is within {@link LimelightConstants#REEF_TRUST_RANGE} of the robot.
     * @return Whether position data can be trusted.
     */
    public boolean canTrustPosition() {
        if (this.canTrustPosition == null) {
            this.canTrustPosition =
                this.MegaTag2 != null
                && (
                    this.MegaTag2.tagCount > 0 
                    && (
                        this.MegaTag2.avgTagDist < LimelightConstants.TRUST_RANGE
                        // Trust any data when disabled, so that it can set initial position / rotation.
                        || DriverStation.isDisabled()
                    )
                );
        }
        return this.canTrustPosition;
    }

    public double calculatePositionDeviation() {
        if (this.MegaTag2.avgTagDist <= LimelightConstants.REEF_TRUST_RANGE) {
            return 0.65;
        }
        else if (this.MegaTag2.avgTagDist <= 3 || DriverStation.isDisabled()) {
            return 1.2;
        }
        else if (this.MegaTag2.avgTagDist <= LimelightConstants.TRUST_RANGE) {
            return 1.8;
        }
        else {
            return 9999999;
        }
    }

    public double calculateRotationDeviation() {
        if (this.MegaTag2.avgTagDist <= LimelightConstants.REEF_TRUST_RANGE
            || DriverStation.isDisabled()
        ) {
            return Units.degreesToRadians(35);
        }
        else {
            return 9999999;
        }
    }
}
