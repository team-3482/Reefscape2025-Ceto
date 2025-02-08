package frc.robot.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

/** A command which lifts the elevator and lets it fall down slowly to reset position. */
public class ZeroElevatorCommand extends Command {
    private Timer liftTimer = new Timer();

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
        ElevatorSubsystem.getInstance().setVoltage(ElevatorConstants.ZERO_LIFT_VOLTAGE);
        liftTimer.start();
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        ElevatorSubsystem.getInstance().setVoltage(0);
    }


    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return liftTimer.hasElapsed(ElevatorConstants.ZERO_LIFT_TIME);
    }
}
