// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.coral;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.led.StatusColors;
import frc.robot.led.LEDSubsystem;

/** A command that ejects the coral and stops a short time after. */
public class OuttakeCoralCommand extends Command {
    private Timer timer = new Timer();

    /** Creates a new OuttakeCoralCommand. */
    public OuttakeCoralCommand() {
        setName("OuttakeCoralCommand");
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(CoralSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        CoralSubsystem.getInstance().outtake();
        this.timer.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Starts the timer once the note doesn't break the beam
        if (!CoralSubsystem.getInstance().hasCoral()) {
            this.timer.start();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        CoralSubsystem.getInstance().stop();
        this.timer.stop();
        LEDSubsystem.getInstance().setColor(StatusColors.OK);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Stops the motor after 0.25 seconds of the coral leaving the beam
        return timer.hasElapsed(0.35);
    }
}
