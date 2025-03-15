package frc.robot.elevator;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.ScoringConstants;

/** A command that moves the elevator to a position. */
public class MoveElevatorCommand extends Command {
    private final double position;
    private final Supplier<Boolean> slowSupplier;
    private final boolean toggle;

    /**
     * Creates a new ElevatorCommand.
     * @param position - The position the elevator will move to. In meters
     * @param slowSupplier - The supplier for running the elevator slower.
     * @param toggle - When the command ends, the elevator will return to the bottom position.
     * It will also never end, such that it stays toggled.
     */
    public MoveElevatorCommand(double position, Supplier<Boolean> slowSupplier, boolean toggle) {
        setName("ElevatorCommand");
        
        this.position = position;
        this.slowSupplier = slowSupplier;
        this.toggle = toggle;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(ElevatorSubsystem.getInstance());
    }

    /**
     * Creates a new ElevatorCommand.
     * @param position - The position the elevator will move to in meters.
     * @param slow - Whether to move the elevator slower.
     * @param toggle - When the command ends, the elevator will return to the bottom position.
     * It will also never end, such that it stays toggled.
     */
    public MoveElevatorCommand(double position, boolean slow, boolean toggle) {
        this(position, () -> slow, toggle); // Still creates a supplier that supplies just the boolean
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
    public void end(boolean interrupted) {
        if (this.toggle) {
            ElevatorSubsystem.getInstance().motionMagicPosition(
                ScoringConstants.BOTTOM_HEIGHT, true, this.slowSupplier.get()
            );
        }
    }


    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !this.toggle && ElevatorSubsystem.getInstance().withinTolerance(this.position);
    }
}
