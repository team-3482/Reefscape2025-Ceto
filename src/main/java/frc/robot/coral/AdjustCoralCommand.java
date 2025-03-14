// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.coral;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.led.StatusColors;
import frc.robot.led.LEDSubsystem;

/** A command that adjusts the coral in case it has been intook too far. */
public class AdjustCoralCommand extends Command {
    private final Timer timer = new Timer();
    private boolean atBackOnce = false;

    // TODO AT COMP : Test this command in the pits
    /** Creates a new AdjustCoralCommand. */
    public AdjustCoralCommand() {
        setName("AdjustCoralCommand");
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(CoralSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.atBackOnce = false;
        this.timer.restart();
        CoralSubsystem.getInstance().slowIntake(-1);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!this.atBackOnce && CoralSubsystem.getInstance().hasCoral_backLaser()) {
            this.atBackOnce = true;
            CoralSubsystem.getInstance().slowIntake(1);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        CoralSubsystem.getInstance().stop();
        LEDSubsystem.getInstance().setColor(interrupted ? StatusColors.ERROR : StatusColors.OK);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.timer.hasElapsed(1)
            && CoralSubsystem.getInstance().hasCoral_frontLaser()
            && !CoralSubsystem.getInstance().hasCoral_backLaser();
    }
}
