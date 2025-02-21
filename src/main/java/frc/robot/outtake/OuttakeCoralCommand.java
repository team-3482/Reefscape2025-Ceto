// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.outtake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that does nothing. */
public class OuttakeCoralCommand extends Command {
    private Timer timer = new Timer();

    /**
     * Creates a new ExampleCommand.
     */
    public OuttakeCoralCommand() {
        setName("OuttakeCoralCommand");
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(CoralSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    /**
     * Sets the motors to the outtake speed
     */
    @Override
    public void initialize() {
        CoralSubsystem.getInstance().outtake();
        this.timer.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    /**
     * Starts the timer once the note doesn't break the beam
     */
    @Override
    public void execute() {
        if (!CoralSubsystem.getInstance().hasCoral()) {
            this.timer.start();
        }
    }

    // Called once the command ends or is interrupted.
    /**
     * Stops the motor after 0.1 seconds of the coral escaping the beam
     */
    @Override
    public void end(boolean interrupted) {
        CoralSubsystem.getInstance().stop();
        this.timer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(0.1);
    }
}
