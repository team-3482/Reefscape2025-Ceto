package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.algae.AlgaeSubsystem;
import frc.robot.swerve.SwerveSubsystem;

/**
 * A class that holds static methods that group Commands under specific names.
 * This is useful because that way RobotContainer.java has less code and more readability.
 */
public final class CommandGenerators {
    // GENERAL
    //
    //
    //
    //
    //
    //
    /**
     * A command that cancels all running commands.
     * @return The command.
     */
    public static Command CancelAllCommands() {
        return Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll());
    }

    // DRIVER
    //
    //
    //
    //
    //
    //
    /**
     * A command that takes the current orientation of the robot
     * and makes it X forward for field-relative maneuvers.
     * @return The command.
     */
    public static Command SetForwardDirectionCommand() {
        return Commands.runOnce(() -> SwerveSubsystem.getInstance().seedFieldCentric());
    }

    /**
     * A command that resets the odometry to an empty Pose2d.
     * @return The command.
     */
    public static Command ResetOdometryToOriginCommand() {
        return Commands.runOnce(() -> SwerveSubsystem.getInstance().resetPose(Pose2d.kZero));
    }

    // OPERATOR
    //
    //
    //
    //
    //
    //

    // AUTO
    //
    //
    //
    //
    //
    //
    /**
     * A command that outtakes the algae.
     * @return The command.
     */
    public static Command EnableAlgaeCommand() {
        return AlgaeSubsystem.getInstance().runOnce(
            () -> AlgaeSubsystem.getInstance().enable()
        );
    }

    /**
     * A command that stops the algae.
     * @return The command.
     */
    public static Command DisableAlgaeCommand() {
        return AlgaeSubsystem.getInstance().runOnce(
            () -> AlgaeSubsystem.getInstance().stop()
        );
    }
}
