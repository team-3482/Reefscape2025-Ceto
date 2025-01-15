// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
    private Command auton;

    public Robot() {
        RobotContainer robotContainer = RobotContainer.getInstance();
        robotContainer.configureDriverBindings();
        robotContainer.configureOperatorBindings();

        Logger.recordMetadata("ProjectName", "Reefscape2025"); // Set a metadata value

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
            Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
            Logger.recordMetadata("GitDirty", BuildConstants.DIRTY == 1 ? "true" : "false");            
            Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        } else {
            setUseTiming(false); // Run as fast as possible
            String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
            Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        }
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledInit() {
        SignalLogger.stop();
        Logger.end();
    }

    @Override
    public void disabledExit() {
        SignalLogger.start();
        Logger.start();
    }

    @Override
    public void autonomousInit() {
        this.auton = RobotContainer.getInstance().getAutonomousCommand();
        if (this.auton != null) {
            this.auton.schedule();
        }
        else {
            System.err.println("No auton command found.");
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (auton != null) {
            this.auton.cancel();
        }
    } 

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        
        // TODO ROBOT BUILT : Find Max Module Speed
        // CommandSwerveDrivetrain.getInstance().setControl(
        //     new SwerveRequest.SysIdSwerveTranslation().withVolts(12)
        // );
    }

    // private double topSpeed;
    @Override
    public void testPeriodic() {
        // ChassisSpeeds speeds = CommandSwerveDrivetrain.getInstance().getState().Speeds;
        // double speed = Math.sqrt(Math.pow(speeds.vyMetersPerSecond, 2) + Math.pow(speeds.vxMetersPerSecond, 2));
        // topSpeed = Math.max(this.topSpeed, speed);
        // System.out.println(this.topSpeed);
    }

    @Override
    public void testExit() {
        // CommandSwerveDrivetrain.getInstance().setControl(new SwerveRequest.SwerveDriveBrake());
    }
}
