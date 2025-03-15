// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Inspiration taken from https://github.com/MAikenMagic1102/2023SheriffPhoenix6/blob/main/src/main/java/frc/robot/commands/PIDdriveToNearestGrid.java

package frc.robot.auto;

import java.util.Optional;
import java.util.function.Function;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.AligningConstants;
import frc.robot.constants.Constants.TagSets;
import frc.robot.led.StatusColors;
import frc.robot.led.LEDSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.utilities.FilteredTranslation;
import frc.robot.vision.VisionSubsystem;

/**
 * A command that wraps PID controllers to align to a position relative to a tag.
 */
public class PIDAlignCommand extends Command {
    private final static ChassisSpeeds ZERO_SPEEDS = new ChassisSpeeds();
    private final SwerveRequest.ApplyRobotSpeeds drive = new SwerveRequest.ApplyRobotSpeeds();

    private final Function<Integer, Boolean> ALL_TAGS;
    private final Function<Integer, Boolean> FLIPPED_TAGS;
    private final double PERPENDICULAR_DIST_TO_TAG;
    private final double PARALLEL_DIST_TO_TAG;

    private Pose2d targetPose;
    private FilteredTranslation targetTranslation;

    private final PIDController xController = new PIDController(2.3, 0, 0);
    private final PIDController yController = new PIDController(2.3, 0, 0);
    private final PIDController thetaController = new PIDController(2.7, 0, 0);
    private final double kF_linear = 0.075;
    private final double kF_angular = Math.PI / 20;

    private double perpendicularError = 0;
    private double parallelError = 0;
    private boolean firstPose = false;

    private final Timer timer = new Timer();

    /**
     * Creates a new PIDAlignCommand.
     * @param name - The name of the sub-command.
     * @param normalTags - A function that checks if a tag can be aligned to.
     * @param allTags - Tags, if any, that the robot should flip the direction for.
     * This is useful for the driver.
     * @param direction - Line up to the left (-1), right (1), or center of the tag.
     * @param perpendicularDistanceToTag - The perpendicular distance from the tag to line up.
     * @param parallelDistanceToTag  - The parallel distance from the tag to line up.
     */
    private PIDAlignCommand(
        String name, Function<Integer, Boolean> normalTags, Function<Integer, Boolean> allTags,
        int direction, double perpendicularDistanceToTag, double parallelDistanceToTag
    ) {
        setName(name);

        this.ALL_TAGS = normalTags;
        this.FLIPPED_TAGS = allTags;
        this.PERPENDICULAR_DIST_TO_TAG = perpendicularDistanceToTag;
        this.PARALLEL_DIST_TO_TAG = parallelDistanceToTag * Math.signum(direction);

        this.xController.setTolerance(0.015);
        this.yController.setTolerance(0.015);
        this.thetaController.setTolerance(Units.degreesToRadians(2));
        this.thetaController.enableContinuousInput(0, 2 * Math.PI);

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(SwerveSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.targetPose = Pose2d.kZero;
        this.firstPose = true;
        
        this.parallelError = Double.NaN;
        this.perpendicularError = Double.NaN;

        this.xController.reset();
        this.yController.reset();
        this.thetaController.reset();

        this.timer.restart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Pose2d newPose = getTargetPose().orElse(Pose2d.kZero);
        if (this.targetPose == Pose2d.kZero && newPose == Pose2d.kZero) return;
        
        if (this.firstPose) {
            this.targetPose = newPose;
            this.targetTranslation = new FilteredTranslation(newPose.getTranslation());
            this.firstPose = false;
        }
        else if (newPose != Pose2d.kZero) {
            this.targetPose = new Pose2d(
                this.targetTranslation.getNextTranslation(newPose.getTranslation()),
                this.targetPose.getRotation()
            );
        }

        Pose2d currentPose = SwerveSubsystem.getInstance().getState().Pose;

        double xSpeed = this.xController.calculate(currentPose.getX(), this.targetPose.getX());
        double ySpeed = this.yController.calculate(currentPose.getY(), this.targetPose.getY());
        double thetaSpeed = this.thetaController.calculate(
            currentPose.getRotation().getRadians(),
            this.targetPose.getRotation().getRadians()
        );

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed + Math.signum(xSpeed) * (this.kF_linear),
            ySpeed + Math.signum(ySpeed) * (this.kF_linear),
            thetaSpeed + Math.signum(thetaSpeed) * (this.kF_angular),
            currentPose.getRotation()
        );
        
        SwerveSubsystem.getInstance().setControl(this.drive.withSpeeds(speeds));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        SwerveSubsystem.getInstance().setControl(this.drive.withSpeeds(PIDAlignCommand.ZERO_SPEEDS));

        // The latter doesn't interrupt the Command even though it's an early end
        if (interrupted || this.targetPose == Pose2d.kZero) {
            LEDSubsystem.getInstance().setColor(StatusColors.ERROR);
        }
        else {
            LEDSubsystem.getInstance().setColor(StatusColors.OK);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // System.out.println(
        //     this.targetPose.equals(Pose2d.kZero) + " && " + this.timer.hasElapsed(0.25)
        //     + " || " + (Math.abs(this.parallelError) <= 0.025) 
        //     + " && " + (Math.abs(this.perpendicularError) <= 0.025)
        //     + " && " + (Math.abs(this.thetaController.getError()) <= Units.degreesToRadians(3))
        // );

        // System.out.println("parallel " + this.parallelError + " perpe " + this.perpendicularError);
        return (this.targetPose == Pose2d.kZero && this.timer.hasElapsed(0.25))
            || (
                Math.abs(this.parallelError) <= 0.02
                && Math.abs(this.perpendicularError) <= 0.02
                && Math.abs(this.thetaController.getError()) <= Units.degreesToRadians(3)
            );
    }

    /**
     * Gets the target pose.
     * @return The target pose.
     */
    private Optional<Pose2d> getTargetPose() {
        Optional<Pose2d> botPose_TargetSpace = VisionSubsystem.getInstance().getEstimatedPosition_TargetSpace();
        int primaryInViewTag = VisionSubsystem.getInstance().getPrimaryTagInView_Bottom_MegaTag();

        if (
            botPose_TargetSpace.isEmpty() ||
            !VisionSubsystem.getInstance().recentVisionData() ||
            !this.ALL_TAGS.apply(primaryInViewTag)
        ) {
            return Optional.empty();
        }
    
        /** Forwards (from tag perspective, closer) is positive. */
        double perpendicularChange = -(botPose_TargetSpace.get().getY() + this.PERPENDICULAR_DIST_TO_TAG);
        this.perpendicularError = perpendicularChange;
        /** Right (from tag perspective, left) is positive. */
        double parallelChange = (this.FLIPPED_TAGS.apply(primaryInViewTag) ? -1 : 1) *
            this.PARALLEL_DIST_TO_TAG - botPose_TargetSpace.get().getX();
        this.parallelError = parallelChange;

        Pose2d botPose = SwerveSubsystem.getInstance().getState().Pose;

        Rotation2d targetRotation = botPose.getRotation().plus(botPose_TargetSpace.get().getRotation());
        targetRotation = Rotation2d.fromDegrees(roundToHexagonalNormal(targetRotation.getDegrees()));
        
        double xChange = targetRotation.getCos() * perpendicularChange + targetRotation.plus(Rotation2d.kCW_Pi_2).getCos() * parallelChange;
        double yChange = targetRotation.getSin() * perpendicularChange + targetRotation.plus(Rotation2d.kCW_Pi_2).getSin() * parallelChange;

        if (Math.hypot(xChange, yChange) > 1) {
            // Don't try this command farther than 1 meter from the goal.
            return Optional.empty();
        }

        return Optional.of(new Pose2d(
            botPose.getTranslation().plus(new Translation2d(xChange, yChange)),
            targetRotation
        ));
    }

    /**
     * Rounds the input angle to the closest normal to a hexagon's face.
     * @param angle - The angle to round.
     * @return The rounded angle.
     * @implSpec Use degrees.
     */
    private int roundToHexagonalNormal(double angle) {
        angle = angle % 360;
        return (int) ((Math.round(angle / 60.0) * 60) % 360);
    }
    
    /**
     * A command that extends a PIDAlignCommand to align to the reef.
     */
    public static class Reef extends PIDAlignCommand {
        /**
         * Creates a new AlignToReefCommand.
         * @param direction - Line up to the left (-1), right (1), or center of the tag.
         * @param flipTags - Whether to use the set of tags that flip the direction.
         */
        public Reef(int direction, boolean flipTags) {
            super(
                "AlignToReefCommand",
                TagSets.REEF_TAGS::contains,
                (flipTags ? TagSets.REEF_TAGS_FLIPPED::contains : TagSets.EMPTY_SET::contains),
                direction,
                direction == 0
                    ? AligningConstants.Reef.PERPENDICULAR_DIST_TO_TAG_ALGAE
                    : AligningConstants.Reef.PERPENDICULAR_DIST_TO_TAG_CORAL,
                AligningConstants.Reef.PARALLEL_DIST_TO_TAG
            );
        }
    }

    /**
     * A command that extends a PIDAlignCommand to align to the processor.
     */
    public static class Processor extends PIDAlignCommand {
        /**
         * Creates a new AlignToProcessorCommand.
         */
        public Processor() {
            super(
                "AlignToProcessorCommand",
                TagSets.PROCESSOR_TAGS::contains,
                TagSets.EMPTY_SET::contains,
                0, // Doesn't matter, because the parallel distance is 0.
                AligningConstants.Processor.PERPENDICULAR_DIST_TO_TAG,
                AligningConstants.Processor.PARALLEL_DIST_TO_TAG
            );
        }
    }
}