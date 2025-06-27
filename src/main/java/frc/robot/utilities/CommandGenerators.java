package frc.robot.utilities;

import static edu.wpi.first.units.Units.Meters;

import java.text.DecimalFormat;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.algae.AlgaeSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.utilities.Elastic.Notification;
import frc.robot.utilities.Elastic.Notification.NotificationLevel;
import frc.robot.vision.VisionSubsystem;

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

    // AUTO
    //
    //
    //
    //
    //
    //
    /**
     * A command that waits for limelights to be close to odometry before finishing.
     * @param The PIDAlign Command to run after waiting.
     * @return The command.
     */
    public static Command WaitForLimelightsCommand(Command PIDAlignCommand) {
        final DecimalFormat DOUBLE_FORMAT = new DecimalFormat("#.##");
        Timer timeoutTimer = new Timer();

        return Commands.sequence(
            Commands.runOnce(() -> timeoutTimer.restart()),

            Commands.waitUntil(() -> {
                VisionSubsystem.getInstance().waitingForLimelights = true;

                if (!timeoutTimer.hasElapsed(DriverStation.isTeleop() ? 0.5 : 1)) {
                    return SwerveSubsystem.getInstance().getDistance(
                        VisionSubsystem.getInstance().getPose2d().pose2d.getTranslation()
                    ).in(Meters) <= 0.1;
                }
                else {
                    String wastedTime = DOUBLE_FORMAT.format(timeoutTimer.get());
                    Logger.recordOutput("PIDAlign Wasted Time", wastedTime);
                    
                    String wastedTimeMsg = "Wasted "
                        + wastedTime
                        + " seconds waiting for Limelights"; 
                    System.out.println(wastedTimeMsg);
                    Elastic.sendNotification(new Notification(NotificationLevel.WARNING, "PIDAlign",  wastedTimeMsg));

                    VisionSubsystem.getInstance().waitingForLimelights = false;
                    return true;
                }
            }),

            PIDAlignCommand
        );
    }
}
