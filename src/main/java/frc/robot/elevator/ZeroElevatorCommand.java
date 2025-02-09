package frc.robot.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PhysicalConstants.ElevatorConstants;

/** A command which lifts the elevator and lets it fall down slowly to reset position. */
public class ZeroElevatorCommand extends Command {
    /**
     * Creates a new ZeroElevatorCommand.
     */
    public ZeroElevatorCommand() {
        setName("ZeroElevatorCommand");
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(ElevatorSubsystem.getInstance());
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        ElevatorSubsystem.getInstance().setVoltage(-1);
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            ElevatorSubsystem.getInstance().motionMagicPosition(ElevatorSubsystem.getInstance().getPosition(), false);
        }
        else {
            ElevatorSubsystem.getInstance().setVoltage(0);
            ElevatorSubsystem.getInstance().setPosition(ElevatorConstants.LOWER_STOP);
        }
    }


    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return ElevatorSubsystem.getInstance().getRotorVelocity() == 0;
    }
}
