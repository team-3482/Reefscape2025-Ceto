package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

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

    // OPERATOR
    //
    //
    //
    //
    //
    //
}