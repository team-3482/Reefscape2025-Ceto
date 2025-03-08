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
import frc.robot.constants.Constants.SubsystemStates;
import frc.robot.constants.PhysicalConstants.AlgaeConstants;
import frc.robot.constants.PhysicalConstants.RobotConstants;

import org.littletonrobotics.junction.Logger;

/** A subsystem that manipulates the Algae game piece. */
public class AlgaeSubsystem extends SubsystemBase {
    // Use Bill Pugh Singleton Pattern for efficient lazy initialization (thread-safe !)
    private static class AlgaeSubsystemHolder {
        private static final AlgaeSubsystem INSTANCE = new AlgaeSubsystem();
    }
    
    public static AlgaeSubsystem getInstance() {
        return AlgaeSubsystemHolder.INSTANCE;
    }

    private TalonFX rightMotor = new TalonFX(AlgaeConstants.RIGHT_MOTOR_ID, RobotConstants.CTRE_CAN_BUS);
    private TalonFX leftMotor = new TalonFX(AlgaeConstants.LEFT_MOTOR_ID, RobotConstants.CTRE_CAN_BUS);

    private SubsystemStates state = SubsystemStates.STOPPED;
    private SubsystemStates lastLoggedState = SubsystemStates.STOPPED;

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
        if (this.state != this.lastLoggedState) {
            Logger.recordOutput("Algae/State", this.state);
            this.lastLoggedState = this.state;
        }
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
        this.state = SubsystemStates.INTAKING;
        rightMotor.setVoltage(AlgaeConstants.INTAKE_OUTTAKE_VOLTAGE);
    }

    /**
     * Sets the speed of the motors to the outtake speed
     */
    public void outtake() {
        this.state = SubsystemStates.OUTTAKING;
        rightMotor.setVoltage(-AlgaeConstants.INTAKE_OUTTAKE_VOLTAGE);
    }

    /**
     * Sets the speed of the motors to the holding voltage.
     */
    public void hold() {
        this.state = SubsystemStates.HOLDING;
        this.rightMotor.setVoltage(AlgaeConstants.HOLDING_VOLTAGE);
    }

    /**
     * Stops the motors immediately
     */
    public void stop() {
        this.state = SubsystemStates.STOPPED;
        rightMotor.setVoltage(0);
    }
}
