// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Inspiration taken from https://github.com/MAikenMagic1102/2023SheriffPhoenix6/blob/main/src/main/java/frc/robot/commands/PIDdriveToNearestGrid.java

package frc.robot.auto;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.AligningConstants;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.vision.VisionSubsystem;

/**
 * A command that wraps a PathPlanner command that paths to the reef for scoring.
 */
public class AlignToReefCommand extends Command {
    private final boolean right;

    private Pose2d targetPose;

    private final PIDController xController = new PIDController(1, 0, 0);
    private final PIDController yController = new PIDController(1, 0, 0);
    private final PIDController thetaController = new PIDController(1, 0, 0);

    private SwerveRequest.ApplyRobotSpeeds drive = new SwerveRequest.ApplyRobotSpeeds(); 

    /**
     * Creates a new AlignToReefCommand.
     * @param right - Whether to align with the left or right.
     */
    public AlignToReefCommand(boolean right) {
        setName("DriveToNoteCommand");

        this.right = right;
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
            double perpendicularChange = botPose_TargetSpace.getY() - AligningConstants.Reef.PERPENDICULAR_DIST_TO_TAG;
            /** Right (from tag perspective, left) is positive. */
            double parallelChange =
                botPose_TargetSpace.getX() + AligningConstants.Reef.PARALLEL_DIST_TO_TAG * (this.right ? 1 : -1);
            
            Pose2d botPose = SwerveSubsystem.getInstance().getState().Pose;
            
            double xChange = botPose.getRotation().getCos() * (perpendicularChange + parallelChange);
            double yChange = -botPose.getRotation().getSin() * (perpendicularChange + parallelChange);
    
            this.targetPose = new Pose2d(
                botPose.getTranslation().plus(new Translation2d(xChange, yChange)),
                botPose.getRotation().plus(botPose_TargetSpace.getRotation())
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
        );
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
}