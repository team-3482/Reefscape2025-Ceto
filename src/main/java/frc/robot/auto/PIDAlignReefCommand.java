// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Inspiration taken from https://github.com/MAikenMagic1102/2023SheriffPhoenix6/blob/main/src/main/java/frc/robot/commands/PIDdriveToNearestGrid.java

package frc.robot.auto;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AprilTagMap;
import frc.robot.constants.LimelightConstants;
import frc.robot.constants.Constants.AligningConstants;
import frc.robot.constants.Constants.TagSets;
import frc.robot.led.LEDSubsystem;
import frc.robot.led.StatusColors;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.vision.VisionSubsystem;

/**
 * A command that wraps PID controllers to align to a position relative to a tag.
 */
public class PIDAlignReefCommand extends Command {
    private final static ChassisSpeeds ZERO_SPEEDS = new ChassisSpeeds();
    private final SwerveRequest.ApplyRobotSpeeds drive = new SwerveRequest.ApplyRobotSpeeds(); 
    private final double[] ZERO_ARRAY = new double[] { 0, 0, 0 };
    
    private final int direction;
    private final boolean flipTags;
    private final boolean coralFirst;
    private final boolean waitForLimelights;
    
    private Pose2d targetPose;
    private int targetID;

    private Pose2d changePose2d;

    private final PIDController xController = new PIDController(4.3, 0, 0);
    private final PIDController yController = new PIDController(4.3, 0, 0);
    private final PIDController thetaController = new PIDController(4.3, 0, 0);

    private final double kF_linear = 0.065;
    private final double kF_angular = 0.16;

    private final Timer timer;

    /**
     * Creates a new PIDAlignCommand.
     * @param direction - The direction to align, robot-relative when facing the tag (-1, 0, 1).
     * @param flipTags - Whether to flip the tags on the opposite side of the driver to be driver-relative.
     * @param coralFirst - Whether to align further away considering for a coral.
     * @param waitForLimelights - Wait for data to be up-to-date.
     */
    public PIDAlignReefCommand(int direction, boolean flipTags, boolean coralFirst, boolean waitForLimelights) {
        setName("PIDAlignReefCommand");

        this.direction = (int) Math.signum(direction);
        this.flipTags = flipTags;
        this.coralFirst = coralFirst;
        this.waitForLimelights = waitForLimelights;
        this.timer = new Timer();

        this.xController.setTolerance(0.005);
        this.yController.setTolerance(0.005);
        this.thetaController.setTolerance(Units.degreesToRadians(1.5));
        this.thetaController.enableContinuousInput(0, 2 * Math.PI);

        this.xController.setSetpoint(0);
        this.yController.setSetpoint(0);
        this.thetaController.setSetpoint(0);

        SmartDashboard.putNumberArray("Aligning Target Pose", VisionSubsystem.pose2dToArray(Pose2d.kZero));

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(SwerveSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.targetID = VisionSubsystem.getInstance().getPrimaryTagInView();
        this.targetPose = AprilTagMap.calculateReefAlignedPosition(
            this.targetID,
            this.direction * (this.flipTags && TagSets.REEF_TAGS_FLIPPED.contains(this.targetID) ? -1 : 1)
        );

        if (
            SwerveSubsystem.getInstance().getDistance(this.targetPose.getTranslation())
            .in(Meters) >= LimelightConstants.REEF_ALIGN_RANGE
        ) {
            this.targetPose = Pose2d.kZero;
        }
        
        SmartDashboard.putNumberArray("Aligning Target Pose", VisionSubsystem.pose2dToArray(this.targetPose));
        
        this.changePose2d = null;
        this.xController.reset();
        this.yController.reset();
        this.thetaController.reset();

        this.timer.restart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (this.targetPose == Pose2d.kZero) return;

        this.changePose2d = calculateChangeRobotRelative();

        // These are negative because the setpoint is zero.
        // To go from 0 to +x, you need a positive speed. If the setpoint is zero, you are doing +x to 0.
        double xSpeed = -this.xController.calculate(this.changePose2d.getX());
        if (this.xController.atSetpoint()) {
            // Do this because around the tolerance PID is not zero,
            // and combined with the friction speed this could mess up other directions.
            xSpeed = 0; 
        }

        double ySpeed = -this.yController.calculate(this.changePose2d.getY());
        if (this.yController.atSetpoint()) {
            ySpeed = 0;
        }
        
        double thetaSpeed = -this.thetaController.calculate(this.changePose2d.getRotation().getRadians());
        if (this.thetaController.atSetpoint()) {
            thetaSpeed = 0;
        }

        SmartDashboard.putNumberArray("Aligning Change Needed", VisionSubsystem.pose2dToArray(changePose2d));
        
        ChassisSpeeds speeds = new ChassisSpeeds(
            xSpeed + Math.signum(xSpeed) * this.kF_linear,
            ySpeed + Math.signum(ySpeed) * this.kF_linear,
            thetaSpeed + Math.signum(thetaSpeed) * this.kF_angular
        );

        SmartDashboard.putNumberArray("Aligning Speeds", new Double[] {
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            Math.toDegrees(speeds.omegaRadiansPerSecond)
        });
        
        SwerveSubsystem.getInstance().setControl(this.drive.withSpeeds(speeds));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        SwerveSubsystem.getInstance().setControl(this.drive.withSpeeds(PIDAlignReefCommand.ZERO_SPEEDS));
        SmartDashboard.putNumberArray("Aligning Target Pose", this.ZERO_ARRAY);
        SmartDashboard.putNumberArray("Aligning Change Needed", this.ZERO_ARRAY);
        SmartDashboard.putNumberArray("Aligning Speeds", this.ZERO_ARRAY);

        // The latter doesn't interrupt the Command even though it's an early end
        if (interrupted || this.targetPose == Pose2d.kZero) {
            LEDSubsystem.getInstance().setColor(StatusColors.ERROR);
        }
        else {
            LEDSubsystem.getInstance().setColor(StatusColors.OK);
        }

        System.out.println("\n\nTime taken to align : " + this.timer.get() + "\n\n");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (this.targetPose == Pose2d.kZero && this.timer.hasElapsed(0.25))
            || (
                this.changePose2d != null
                && this.xController.atSetpoint()
                && this.yController.atSetpoint()
                && Math.abs(this.thetaController.getError()) <= Units.degreesToRadians(5)
            );
    }

    /**
     * The distances needed to drive to get to the target pose.
     * Essentially just the values to use with the PIDControllers.
     * @return The change pose.
     */
    private Pose2d calculateChangeRobotRelative() {
        Pose2d robotPose = SwerveSubsystem.getInstance().getState().Pose;
        double robotCos = robotPose.getRotation().getCos();
        double robotSin = robotPose.getRotation().getSin();

        double xChange = this.targetPose.getX() - robotPose.getX();
        double yChange = this.targetPose.getY() - robotPose.getY();

        return new Pose2d(
            new Translation2d(
                xChange * robotCos + yChange * robotSin,
                -xChange * robotSin + yChange * robotCos
            ).minus(
                this.coralFirst
                    ? new Translation2d(AligningConstants.Reef.CORAL_DISTANCE, 0)
                    : Translation2d.kZero
            ),
            this.targetPose.getRotation().minus(robotPose.getRotation())
        );
    }
}
