// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.StatusColors;
import frc.robot.led.LEDSubsystem;

/** A command that intakes coral and stops when it reaches the end of the intake. */
public class IntakeCoralCommand extends Command {

    /** Creates a new IntakeCoralCommand. */
    public IntakeCoralCommand() {
        setName("IntakeCoralCommand");
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(CoralSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        CoralSubsystem.getInstance().intake();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Checks whether the system has a note and stops if it does
        if (CoralSubsystem.getInstance().hasCoral_frontLaser()) {
            CoralSubsystem.getInstance().slowIntake();
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
        return CoralSubsystem.getInstance().hasCoral_frontLaser() && !CoralSubsystem.getInstance().hasCoral_backLaser();
    }
}
