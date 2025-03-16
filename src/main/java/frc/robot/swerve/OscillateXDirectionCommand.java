package frc.robot.swerve;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** A command that moves the robot back and worth in an oscillatory manner. */
public class OscillateXDirectionCommand extends Command {
    private final static ChassisSpeeds ZERO_SPEEDS = new ChassisSpeeds();
    private final static ChassisSpeeds FORWARD_SPEEDS = new ChassisSpeeds(0.6, 0, 0);
    private final static ChassisSpeeds BACKWARDS_SPEEDS = new ChassisSpeeds(-0.6, 0, 0);
    private final SwerveRequest.ApplyRobotSpeeds drive = new SwerveRequest.ApplyRobotSpeeds();

    private final Timer timer = new Timer();
    private boolean forward;

    /** Creates a new OscillateXDirectionCommand. */
    public OscillateXDirectionCommand() {
        setName("OscillateXDirectionCommand");

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(SwerveSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.timer.restart();
        this.forward = true;
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (this.timer.hasElapsed(0.15)) {
            if (this.forward) {
                SwerveSubsystem.getInstance().setControl(
                    this.drive.withSpeeds(OscillateXDirectionCommand.FORWARD_SPEEDS)
                    );
                }
            else {
                SwerveSubsystem.getInstance().setControl(
                    this.drive.withSpeeds(OscillateXDirectionCommand.BACKWARDS_SPEEDS)
                );
            }
            this.timer.reset();
            this.forward = !this.forward;
        }
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        SwerveSubsystem.getInstance().setControl(this.drive.withSpeeds(OscillateXDirectionCommand.ZERO_SPEEDS));
    }


    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
