// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command auton;

    public Robot() {
        RobotContainer robotContainer = RobotContainer.getInstance();
        robotContainer.configureDriverBindings();
        robotContainer.configureOperatorBindings();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

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
