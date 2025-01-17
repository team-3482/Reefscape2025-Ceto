// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.outtake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PhysicalConstants.OuttakeConstants;;

public class OuttakeSubsystem extends SubsystemBase {
    // Thread-safe singleton design pattern.
    private static volatile OuttakeSubsystem instance;
    private static Object mutex = new Object();

    public static OuttakeSubsystem getInstance() {
        OuttakeSubsystem result = instance;
        
        if (result == null) {
            synchronized (mutex) {
                result = instance;
                if (result == null) {
                    instance = result = new OuttakeSubsystem();
                }
            }
        }
        return instance;
    }

    private TalonFX rightMotor = new TalonFX(OuttakeConstants.RIGHT_MOTOR_ID);
    private TalonFX leftMotor = new TalonFX(OuttakeConstants.LEFT_MOTOR_ID); 
    private DigitalInput frontLaser = new DigitalInput(OuttakeConstants.FRONT_LASER_ID);

    /** Creates a new OuttakeSubsystem. */
    private OuttakeSubsystem() {
        super("OuttakeSubsystem");
        configureMotors();

        Follower follow = new Follower(OuttakeConstants.RIGHT_MOTOR_ID, true);
        this.leftMotor.setControl(follow);
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {}

    /**
     * Configures the gear ratio of the motors to our gear ratio
     */
    private void configureMotors() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();
        configuration.Feedback.SensorToMechanismRatio = OuttakeConstants.MECHANISM_RATIO;

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
        rightMotor.setVoltage(OuttakeConstants.INTAKE_VOLTAGE);
    }

    /**
     * Sets the speed of the motors to the outtake speed
     */
    public void outtake() {
        rightMotor.setVoltage(OuttakeConstants.OUTTAKE_VOLTAGE);
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
