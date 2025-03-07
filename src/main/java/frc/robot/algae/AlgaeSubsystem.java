// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.algae;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PhysicalConstants.AlgaeConstants;
import frc.robot.constants.PhysicalConstants.RobotConstants;
import org.littletonrobotics.junction.Logger;

/** A subsystem that manipulates the Algae game piece. */
public class AlgaeSubsystem extends SubsystemBase {
    // Thread-safe singleton design pattern.
    private static volatile AlgaeSubsystem instance;
    private static Object mutex = new Object();

    public static AlgaeSubsystem getInstance() {
        AlgaeSubsystem result = instance;
        
        if (result == null) {
            synchronized (mutex) {
                result = instance;
                if (result == null) {
                    instance = result = new AlgaeSubsystem();
                }
            }
        }
        return instance;
    }

    private TalonFX rightMotor = new TalonFX(AlgaeConstants.RIGHT_MOTOR_ID, RobotConstants.CTRE_CAN_BUS);
    private TalonFX leftMotor = new TalonFX(AlgaeConstants.LEFT_MOTOR_ID, RobotConstants.CTRE_CAN_BUS);

    private String state = "stopped";

    /** Creates a new AlgaeSubsystem. */
    private AlgaeSubsystem() {
        super("AlgaeSubsystem");
        configureMotors();

        Follower follow = new Follower(AlgaeConstants.RIGHT_MOTOR_ID, true);
        this.leftMotor.setControl(follow);
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        Logger.recordOutput("Algae/State", state);
    }

    /**
     * Configures the gear ratio of the motors to our gear ratio
     */
    private void configureMotors() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();
        configuration.Feedback.SensorToMechanismRatio = AlgaeConstants.MECHANISM_RATIO;

        configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        CurrentLimitsConfigs currentLimitsConfigs = configuration.CurrentLimits;
        currentLimitsConfigs.StatorCurrentLimitEnable = true;
        currentLimitsConfigs.StatorCurrentLimit = 100;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        currentLimitsConfigs.SupplyCurrentLimit = 25;
        currentLimitsConfigs.SupplyCurrentLowerTime = 0.5;
        currentLimitsConfigs.SupplyCurrentLowerLimit = 10;

        configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        this.rightMotor.getConfigurator().apply(configuration);
        configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        this.leftMotor.getConfigurator().apply(configuration);
    }

    /**
     * Sets the speed of the motors to the intake speed
     */
    public void intake() {
        rightMotor.setVoltage(AlgaeConstants.INTAKE_OUTTAKE_VOLTAGE);
        state = "intaking";
    }

    /**
     * Sets the speed of the motors to the outtake speed
     */
    public void outtake() {
        rightMotor.setVoltage(-AlgaeConstants.INTAKE_OUTTAKE_VOLTAGE);
        state = "outtaking";
    }

    /**
     * Sets the speed of the motors to the holding voltage.
     */
    public void hold() {
        this.rightMotor.setVoltage(AlgaeConstants.HOLDING_VOLTAGE);
        state = "holding";
    }

    /**
     * Stops the motors immediately
     */
    public void stop() {
        rightMotor.setVoltage(0);
        state = "stopped";
    }
}
