// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PhysicalConstants.QuestNavConstants;
import frc.robot.swerve.SwerveSubsystem;
import gg.questnav.questnav.QuestNav;

/** A subsystem to manage QuestNav vision */
public class QuestNavSubsystem extends SubsystemBase {
    // Use Bill Pugh Singleton Pattern for efficient lazy initialization (thread-safe !)
    private static class QuestNavSubsystemHolder {
        private static final QuestNavSubsystem INSTANCE = new QuestNavSubsystem();
    }

    public static QuestNavSubsystem getInstance() {
        return QuestNavSubsystemHolder.INSTANCE;
    }

    QuestNav questNav = new QuestNav();
    SwerveSubsystem swerveDriveSubsystem = SwerveSubsystem.getInstance();

    /** Creates a new QuestNavSubsystem. */
    private QuestNavSubsystem() {
        super("QuestNavSubsystem");
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        questNav.commandPeriodic();

        if (questNav.isConnected() && questNav.isTracking()) {
            // get starting pose from Limelight (AprilTag) and offset it
            Pose2d robotPose = new Pose2d( /* Some pose data */ ); // TODO limelight data
            Pose2d questPose = robotPose.transformBy(QuestNavConstants.QUEST_OFFSET);

            questNav.setPose(questPose);

            // Get timestamp from the QuestNav instance
            double timestamp = questNav.getDataTimestamp();

            // Add the measurement to our estimator
            swerveDriveSubsystem.addVisionMeasurement(questNav.getPose(), timestamp, QuestNavConstants.TRUST_STD_DEVS);
        }
    }
}
