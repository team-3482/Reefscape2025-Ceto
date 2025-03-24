package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.algae.AlgaeSubsystem;
import frc.robot.constants.Constants.ScoringConstants;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.elevator.MoveElevatorCommand;
import frc.robot.elevator.ZeroElevatorCommand;
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
     * A command that intakes the algae for a certain amount of time before holding.
     * @return The command.
     */
    public static Command IntakeAlgaeAndHoldCommand() {
        return AlgaeSubsystem.getInstance().runOnce(() -> AlgaeSubsystem.getInstance().intake())
            .andThen(Commands.waitSeconds(0.7))
            .andThen(AlgaeSubsystem.getInstance().runOnce(() -> AlgaeSubsystem.getInstance().hold()));
    }

    /**
     * A command that outtakes the algae for a certain amount of time before stopping.
     * @return The command.
     */
    public static Command OuttakeAlgaeAndStopCommand() {
        return AlgaeSubsystem.getInstance().runOnce(() -> AlgaeSubsystem.getInstance().outtake())
            .andThen(Commands.waitSeconds(1))
            .andThen(AlgaeSubsystem.getInstance().runOnce(() -> AlgaeSubsystem.getInstance().stop()));
    }

    /**
     * A command that moves the elevator up to release the algae and re-zero.
     * @return The command.
     */
    public static Command InitialElevatorLiftAndZeroCommand() {
        return Commands.sequence(
            // Just in case it has been booted before the algae was put down,
            // and the position is already correct (should be rare though because it wastes battery)
            ElevatorSubsystem.getInstance().runOnce(() -> ElevatorSubsystem.getInstance().setPosition(0)),
            new MoveElevatorCommand(ScoringConstants.L1_CORAL, false, false),
            new MoveElevatorCommand(ScoringConstants.BOTTOM_HEIGHT, false, false),
            new ZeroElevatorCommand()
        );
    }
}
