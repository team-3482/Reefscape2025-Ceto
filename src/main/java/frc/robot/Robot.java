// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.FollowPathCommand;

import java.io.File;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.*;
import frc.robot.constants.Constants;
import frc.robot.utilities.Elastic;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.led.LEDSubsystem;
import frc.robot.led.StatusColors;

public class Robot extends LoggedRobot {
    private Command auton;
    private final PowerDistribution pdh;

    @SuppressWarnings("unused")
    public Robot() {
        for (int port = 5800; port <= 5809; port++) {
            PortForwarder.add(port, "10.34.82.12", port);
            PortForwarder.add(port + 10, "10.34.82.13", port);
        }

        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

        this.pdh = new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging

        RobotContainer robotContainer = RobotContainer.getInstance();
        robotContainer.configureDriverBindings();
        robotContainer.configureOperatorBindings();

        Logger.recordMetadata("ProjectName", "Reefscape2025"); // Set a metadata value

        if (isReal()) {
            String path = "/U/logs/" + (long)(Math.random() * Math.pow(10, 16));

            System.out.println("logging to: " + path + " (new directory: " + new File(path).mkdirs() + ")");

            SignalLogger.setPath(path);
            
            Logger.addDataReceiver(new WPILOGWriter(path)); // Log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables

            Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
            // This will always be either 0 or 1, so the > sign is used to suppress the comparing identical expressions.
            Logger.recordMetadata("GitDirty", BuildConstants.DIRTY > 0 ? "true" : "false");
            Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);

            Logger.start();
        }

        FollowPathCommand.warmupCommand().schedule();
        // Eager-load the auton command
        RobotContainer.getInstance().getAutonomousCommand();
        
        // Solid orange like the RSL when disabled
        LEDSubsystem.getInstance().setColor(StatusColors.RSL);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        try {
            // TODO : Needs to be tested further to make sure it works properly
            SmartDashboard.putNumber("Total Current", this.pdh.getTotalCurrent());
        }
        catch (Exception error) {
            // Happens when the PDH isn't connected to the RIO.
            // Don't want to spam console during matches.
            error.printStackTrace();
        }

        SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
    }

    @Override
    public void disabledInit() {
        // SignalLogger.stop();
        // Solid orange like the RSL when disabled
        LEDSubsystem.getInstance().setColor(StatusColors.RSL);
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {
        // SignalLogger.start();
        LEDSubsystem.getInstance().setColor(StatusColors.OFF);
    }

    @Override
    public void autonomousInit() {
        this.auton = RobotContainer.getInstance().getAutonomousCommand();

        Logger.recordOutput("Auton/AutonCommand", auton.getName());

        if (this.auton != null) {
            this.auton.schedule();
        }
        else {
            System.err.println("No auton command found.");
            LEDSubsystem.getInstance().setColor(StatusColors.ERROR);
        }

        Elastic.selectTab(Constants.DashboardTabNames.AUTON);
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (auton != null) {
            this.auton.cancel();
            LEDSubsystem.getInstance().setColor(StatusColors.OK);
        }

        Elastic.selectTab(Constants.DashboardTabNames.TELEOP);
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}

// TODO : Clear deploy directory on Rio
// TODO : Tune limelight exposure/gain on field
// TODO : Test alignment, scoring, & removing algae on practice field
