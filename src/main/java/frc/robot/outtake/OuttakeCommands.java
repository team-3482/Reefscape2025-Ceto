// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.outtake;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that does nothing. */
public class OuttakeCommands extends Command {
    /**
     * Creates a new ExampleCommand.
     */
    public OuttakeCommands() {
        setName("OuttakeCommands");
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(OuttakeSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        OuttakeSubsystem.getInstance().intake();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
