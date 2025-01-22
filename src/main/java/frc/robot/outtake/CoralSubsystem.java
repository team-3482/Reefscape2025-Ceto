// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.outtake;

import java.util.Map;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ShuffleboardTabNames;
import frc.robot.constants.PhysicalConstants.CoralConstants;
//import frc.robot.constants.PhysicalConstants.CoralConstants;

public class CoralSubsystem extends SubsystemBase {
    // Thread-safe singleton design pattern.
    private static volatile CoralSubsystem instance;
    private static Object mutex = new Object();

    public static CoralSubsystem getInstance() {
        CoralSubsystem result = instance;
        
        if (result == null) {
            synchronized (mutex) {
                result = instance;
                if (result == null) {
                    instance = result = new CoralSubsystem();
                }
            }
        }
        return instance;
    }

    private TalonFX rightMotor = new TalonFX(CoralConstants.RIGHT_MOTOR_ID);
    private TalonFX leftMotor = new TalonFX(CoralConstants.LEFT_MOTOR_ID); 
    private DigitalInput frontLaser = new DigitalInput(CoralConstants.FRONT_LASER_ID);

    private final ShuffleboardLayout shuffleboardLayout = Shuffleboard.getTab(ShuffleboardTabNames.DEFAULT)
        .getLayout("CoralSubsystem", BuiltInLayouts.kGrid)
        .withProperties(Map.of("Number of columns", 1, "Number of rows", 3, "Label position", "TOP"))
        .withSize(2, 2);
    private GenericEntry shuffleboard_entry = shuffleboardLayout
        .add("Front Laser", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("colorWhenFalse", "black", "colorWhenTrue", "white"))
        .withSize(2, 1)
        .getEntry();

    /** Creates a new OuttakeSubsystem. */
    private CoralSubsystem() {
        super("CoralSubsystem");
        configureMotors();

        Follower follow = new Follower(CoralConstants.RIGHT_MOTOR_ID, true);
        this.leftMotor.setControl(follow);
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        this.shuffleboard_entry.setBoolean(hasCoral());
    }

    /**
     * Configures the gear ratio of the motors to our gear ratio
     */
    private void configureMotors() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();
        configuration.Feedback.SensorToMechanismRatio = CoralConstants.MECHANISM_RATIO;

        configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        this.rightMotor.getConfigurator().apply(configuration);
        configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        this.leftMotor.getConfigurator().apply(configuration);
    }

    /**
     * Sets the speed of the motors to the intake speed
     */
    public void intake() {
        rightMotor.setVoltage(CoralConstants.INTAKE_VOLTAGE);
    }

    /**
     * Sets the speed of the motors to the outtake speed
     */
    public void outtake() {
        rightMotor.setVoltage(CoralConstants.OUTTAKE_VOLTAGE);
    }

    /**
     * Stops the motors immediately
     */
    public void stop() {
        rightMotor.setVoltage(0);
    }

    /**
     * Returns a boolean value of whether there is a coral in the shooter
     * @return whether it has coral
     */
    public boolean hasCoral() {
        return !frontLaser.get();
    }
}
