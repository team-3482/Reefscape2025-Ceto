package frc.robot.elevator;

import edu.wpi.first.wpilibj2.command.Command;

/** A command that moves the elevator to a position. */
public class ElevatorCommand extends Command {
    final double position;

    /**
     * Creates a new ExampleCommand.
     * @param position - The position the elevator will move to. In meters.
     */
    public ElevatorCommand(double position) {
        setName("ElevatorCommand");
        
        this.position = position;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(ElevatorSubsystem.getInstance());
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        ElevatorSubsystem.getInstance().motionMagicPosition(this.position, true);
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}


    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return ElevatorSubsystem.getInstance().withinTolerance(this.position);
    }
}
