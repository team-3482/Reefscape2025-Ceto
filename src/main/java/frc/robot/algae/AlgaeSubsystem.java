// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.algae;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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

    private TalonFX algaeMotor = new TalonFX(AlgaeConstants.ALGAE_MOTOR_ID, RobotConstants.CTRE_CAN_BUS);

    private SubsystemStates state = SubsystemStates.STOPPED;
    private SubsystemStates lastLoggedState = SubsystemStates.STOPPED;

    /** Creates a new AlgaeSubsystem. */
    private AlgaeSubsystem() {
        super("AlgaeSubsystem");
        configureMotors();
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

        configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        CurrentLimitsConfigs currentLimitsConfigs = configuration.CurrentLimits;
        currentLimitsConfigs.StatorCurrentLimitEnable = true;
        currentLimitsConfigs.StatorCurrentLimit = 100;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        currentLimitsConfigs.SupplyCurrentLimit = 50;
        currentLimitsConfigs.SupplyCurrentLowerTime = 0.6;
        currentLimitsConfigs.SupplyCurrentLowerLimit = 20;

        configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        this.algaeMotor.getConfigurator().apply(configuration);
    }

    /**
     * Sets the speed of the motors to the ON speed
     */
    public void enable() {
        this.state = SubsystemStates.DISCARDING;
        algaeMotor.setVoltage(AlgaeConstants.ON_VOLTAGE);
    }

    /**
     * Stops the motors immediately
     */
    public void stop() {
        this.state = SubsystemStates.STOPPED;
        algaeMotor.setVoltage(0);
    }
}
