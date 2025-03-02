// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Inspiration taken from https://github.com/MAikenMagic1102/2023SheriffPhoenix6/blob/main/src/main/java/frc/robot/commands/PIDdriveToNearestGrid.java

package frc.robot.auto;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.AligningConstants;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.vision.VisionSubsystem;

/**
 * A command that wraps PID controllers to align to a position relative to a tag.
 */
public class PIDAlignCommand extends Command {
    private Pose2d targetPose;

    private final PIDController xController = new PIDController(2.5, 0, 0);
    private final PIDController yController = new PIDController(2.5, 0, 0);
    private final PIDController thetaController = new PIDController(3.5, 0, 0);

    private final SwerveRequest.ApplyRobotSpeeds drive = new SwerveRequest.ApplyRobotSpeeds(); 

    private final boolean right;
    private final double PERPENDICULAR_DIST_TO_TAG;
    private final double PARALLEL_DIST_TO_TAG;

    /**
     * Creates a new PIDAlignCommand.
     * @param name - The name of the sub-command.
     * @param right - Whether to line up a parallel distance to the left or right of the tag (robot perspective).
     * @param perpendicularDistanceToTag - The perpendicular distance from the tag to line up.
     * @param parallelDistanceToTag  - The parallel distance from the tag to line up.
     */
    private PIDAlignCommand(String name, boolean right, double perpendicularDistanceToTag, double parallelDistanceToTag) {
        setName(name);

        this.right = right;
        this.PERPENDICULAR_DIST_TO_TAG = perpendicularDistanceToTag;
        this.PARALLEL_DIST_TO_TAG = parallelDistanceToTag;


        this.xController.setTolerance(0.05);
        this.yController.setTolerance(0.05);
        this.thetaController.setTolerance(Math.toRadians(0.5));
        this.thetaController.enableContinuousInput(0, 2 * Math.PI);

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(SwerveSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Pose2d botPose_TargetSpace = VisionSubsystem.getInstance().getEstimatedPosition_TargetSpace();
        if (botPose_TargetSpace == null) {
            this.targetPose = new Pose2d();
            
            this.xController.setSetpoint(0);
            this.yController.setSetpoint(0);
            this.thetaController.setSetpoint(0);
            
            this.xController.calculate(0);
            this.yController.calculate(0);
            this.thetaController.calculate(0);
        }
        else {
            /** Forwards (from tag perspective, closer) is positive. */
            double perpendicularChange = -(botPose_TargetSpace.getY() + this.PERPENDICULAR_DIST_TO_TAG);
            /** Right (from tag perspective, left) is positive. */
            double parallelChange = (this.right ? 1 : -1) * this.PARALLEL_DIST_TO_TAG
                - botPose_TargetSpace.getX();
            Pose2d botPose = SwerveSubsystem.getInstance().getState().Pose;

            Rotation2d targetRotation = botPose.getRotation().plus(botPose_TargetSpace.getRotation());
            
            double xChange = targetRotation.getCos() * perpendicularChange + targetRotation.plus(Rotation2d.kCW_Pi_2).getCos() * parallelChange;
            double yChange = targetRotation.getSin() * perpendicularChange + targetRotation.plus(Rotation2d.kCW_Pi_2).getSin() * parallelChange;

            this.targetPose = new Pose2d(
                botPose.getTranslation().plus(new Translation2d(xChange, yChange)),
                targetRotation
            );

            this.xController.setSetpoint(this.targetPose.getX());
            this.yController.setSetpoint(this.targetPose.getY());
            this.thetaController.setSetpoint(this.targetPose.getRotation().getRadians());
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (this.targetPose.equals(new Pose2d())) return;

        Pose2d currentPose = SwerveSubsystem.getInstance().getState().Pose;
        
        SwerveSubsystem.getInstance().setControl(
            this.drive.withSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                this.xController.calculate(currentPose.getX()),
                this.yController.calculate(currentPose.getY()),
                this.thetaController.calculate(currentPose.getRotation().minus(this.targetPose.getRotation()).getRadians(), 0),
                currentPose.getRotation()
            ))
        ); // TODO : Feedforward default speed to overcome friction ?
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        SwerveSubsystem.getInstance().setControl(this.drive.withSpeeds(new ChassisSpeeds()));

        this.xController.reset();
        this.yController.reset();
        this.thetaController.reset();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.targetPose.equals(new Pose2d()) || (this.xController.atSetpoint() && this.yController.atSetpoint() && this.thetaController.atSetpoint());
    }
    
    /**
     * A command that extends a PIDAlignCommand to align to the reef.
     */
    public static class Reef extends PIDAlignCommand {
        /**
         * Creates a new AlignToReefCommand.
         * @param right - Whether to align to the left or right of the tag (robot perspective).
         */
        public Reef(boolean right) {
            super(
                "AlignToReefCommand",
                right,
                AligningConstants.Reef.PERPENDICULAR_DIST_TO_TAG,
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
                true, // Doesn't matter, because the parallel distance is 0.
                AligningConstants.Reef.PERPENDICULAR_DIST_TO_TAG,
                AligningConstants.Reef.PARALLEL_DIST_TO_TAG
            );
        }
    }
}