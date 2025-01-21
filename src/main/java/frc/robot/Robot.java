// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.Constants.NamedColors;
import frc.robot.led.LEDSubsystem;

public class Robot extends TimedRobot {
    private Command auton;

    public Robot() {
        RobotContainer robotContainer = RobotContainer.getInstance();
        robotContainer.configureDriverBindings();
        robotContainer.configureOperatorBindings();

        LEDSubsystem.getInstance().blinkColor(Color.kOrange);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        // Blink like the RSL when disabled
        LEDSubsystem.getInstance().blinkColor(Color.kOrange);
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {
        LEDSubsystem.getInstance().setColor(Color.kOrange);
    }

    @Override
    public void autonomousInit() {
        this.auton = RobotContainer.getInstance().getAutonomousCommand();
        if (this.auton != null) {
            this.auton.schedule();
            LEDSubsystem.getInstance().setColor(NamedColors.OK);
        }
        else {
            System.err.println("No auton command found.");
            LEDSubsystem.getInstance().blinkColor(NamedColors.ERROR);
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
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
