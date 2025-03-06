package frc.robot.elevator;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

/** A command that moves the elevator to a position. */
public class MoveElevatorCommand extends Command {
    private final double position;
    private final Supplier<Boolean> slowSupplier;

    /**
     * Creates a new ElevatorCommand.
     * @param position - The position the elevator will move to. In meters
     * @param slowSupplier - The supplier for running the elevator slower.
     */
    public MoveElevatorCommand(double position, Supplier<Boolean> slowSupplier) {
        setName("ElevatorCommand");
        
        this.position = position;
        this.slowSupplier = slowSupplier;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(ElevatorSubsystem.getInstance());
    }

    /**
     * Creates a new ElevatorCommand.
     * @param position - The position the elevator will move to in meters.
     * @param slow - Whether to move the elevator slower.
     */
    public MoveElevatorCommand(double position, boolean slow) {
        this(position, () -> slow); // Still creates a supplier that supplies just the boolean
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        ElevatorSubsystem.getInstance().motionMagicPosition(this.position, true, this.slowSupplier.get());
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
