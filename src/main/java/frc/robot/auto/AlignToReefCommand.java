// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.AligningConstants;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.vision.VisionSubsystem;

/**
 * A command that wraps a PathPlanner command that paths to the reef for scoring.
 */
public class AlignToReefCommand extends Command {
    private final PathConstraints constraints = new PathConstraints(
        4, 2,
        Units.degreesToRadians(720), Units.degreesToRadians(540)
    );
    
    private Command pathingCommand;
    private final boolean right;

    /**
     * Creates a new AlignToReefCommand.
     * @param right - Whether to align with the left or right.
     */
    public AlignToReefCommand(boolean right) {
        setName("DriveToNoteCommand");

        this.right = right;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(SwerveSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.pathingCommand = null;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (this.pathingCommand != null) {
            this.pathingCommand.execute();
            return;
        }

        Pose2d botPose_TargetSpace = VisionSubsystem.getInstance().getEstimatedPosition_TargetSpace();
        if (botPose_TargetSpace == null) {
            this.pathingCommand = null;
            return;
        }

        /** Forwards (from tag perspective, closer) is positive. */
        double perpendicularChange = botPose_TargetSpace.getY() - AligningConstants.Reef.PERPENDICULAR_DIST_TO_TAG;
        /** Right (from tag perspective, left) is positive. */
        double parallelChange =
            botPose_TargetSpace.getX() + AligningConstants.Reef.PARALLEL_DIST_TO_TAG * (this.right ? 1 : -1);
        
        Pose2d botPose = SwerveSubsystem.getInstance().getState().Pose;
        Translation2d botTranslation = botPose.getTranslation();
        Rotation2d botRot = botPose.getRotation();
        
        double xChange = botRot.getCos() * (perpendicularChange + parallelChange);
        double yChange = -botRot.getSin() * (perpendicularChange + parallelChange);

        Translation2d targetTranslation = botPose.getTranslation().plus(new Translation2d(xChange, yChange));

        Rotation2d targetRot = botRot.plus(botPose_TargetSpace.getRotation());
        
        this.pathingCommand = generatePath(botTranslation, targetTranslation, targetRot);
        this.pathingCommand.initialize();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println(this.pathingCommand.isFinished());
        if (this.pathingCommand != null) {
            this.pathingCommand.end(interrupted);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.pathingCommand == null || this.pathingCommand.isFinished();
    }

    /**
     * Helper that generates a PathPlanner Command to path to the note's position.
     * @param botTranslation - The robot's current translation.
     * @param targetPose - The position to path to.
     * @param targetRotation - The final rotation to aim for.
     * @return The PathPlanner Command.
     */
    private Command generatePath(Translation2d botTranslation, Translation2d targetTranslation, Rotation2d targetRotation) {
        Rotation2d travelRotation = this.right ? targetRotation.plus(Rotation2d.kCW_Pi_2) : targetRotation.plus(Rotation2d.kCCW_Pi_2);

        Pose2d botPose = new Pose2d(botTranslation, travelRotation);
        Pose2d targetPose = new Pose2d(targetTranslation, travelRotation);

        PathPlannerPath path = new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(botPose, targetPose),
            this.constraints,
            null,
            new GoalEndState(0, targetRotation)
        );
        path.preventFlipping = true; // Field-relative Note position won't change based on alliance
        
        return AutoBuilder.followPath(path);
    }
}