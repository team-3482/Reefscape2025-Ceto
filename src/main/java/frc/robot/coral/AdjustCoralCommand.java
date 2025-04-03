// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.coral;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.led.LEDSubsystem;
import frc.robot.led.StatusColors;
import frc.robot.constants.Constants.ScoringConstants;
import frc.robot.elevator.ElevatorSubsystem;

/** A command that adjusts the coral in case it has been intook too far. */
public class AdjustCoralCommand extends Command {
    private boolean atBackOnce = false;
    private boolean tooHigh = false;

    /** Creates a new AdjustCoralCommand. */
    public AdjustCoralCommand() {
        setName("AdjustCoralCommand");
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(CoralSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.tooHigh = ElevatorSubsystem.getInstance().getPosition() >= ScoringConstants.IDLE_HEIGHT;
        if (this.tooHigh) return;

        this.atBackOnce = false;
        CoralSubsystem.getInstance().reverseIntake(true);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (this.tooHigh) return;

        if (!this.atBackOnce && CoralSubsystem.getInstance().hasCoral_backLaser()) {
            this.atBackOnce = true;
            CoralSubsystem.getInstance().intake(true);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        CoralSubsystem.getInstance().stop();
        LEDSubsystem.getInstance().setColor(interrupted || this.tooHigh ? StatusColors.ERROR : StatusColors.OK);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.tooHigh || (
            this.atBackOnce
            && CoralSubsystem.getInstance().hasCoral_frontLaser()
            && !CoralSubsystem.getInstance().hasCoral_backLaser()
        );
    }
}
