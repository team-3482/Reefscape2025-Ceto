// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.templates;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that does nothing. */
public class ExampleCommand extends Command {
    /** Creates a new ExampleCommand. */
    public ExampleCommand() {
        setName("ExampleCommand");
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(ExampleSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
