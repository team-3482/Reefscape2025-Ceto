// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.AlignToReefCommand;
import frc.robot.constants.Constants.ControllerConstants;
import frc.robot.constants.Constants.ShuffleboardTabNames;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.swerve.SwerveTelemetry;
import frc.robot.swerve.TunerConstants;
import frc.robot.led.LEDSubsystem;
import frc.robot.utilities.CommandGenerators;
import frc.robot.vision.VisionSubsystem;

public class RobotContainer {
    // Thread-safe singleton design pattern.
    private static volatile RobotContainer instance;
    private static Object mutex = new Object();

    public static RobotContainer getInstance() {
        RobotContainer result = instance;
        
        if (result == null) {
            synchronized (mutex) {
                result = instance;
                if (result == null) {
                    instance = result = new RobotContainer();
                }
            }
        }
        return instance;
    }

    private final SendableChooser<Command> autoChooser;
    
    // Instance of the controllers used to drive the robot
    private CommandXboxController driverController;
    private CommandXboxController operatorController;

    public RobotContainer() {
        this.driverController = new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_ID);
        this.operatorController = new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_ID);

        configureDrivetrain(); // This is done separately because it works differently from other Subsystems
        initializeSubsystems();
        
        // Register named commands for Pathplanner (always do this after subsystem initialization)
        registerNamedCommands();

        this.autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be Commands.none()
        Shuffleboard.getTab(ShuffleboardTabNames.DEFAULT)
            .add("Auto Chooser", autoChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser);
    }

    /**
     * This function initializes the swerve subsystem and configures its bindings with the driver controller.
     * This is based on the {@code Phoenix6 Swerve Example} that can be found on GitHub.
     */
    private void configureDrivetrain() {
        final SwerveSubsystem Drivetrain = SwerveSubsystem.getInstance();
        final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond);
        final double LesserMaxSpeed = TunerConstants.kTopSpeed.in(Units.MetersPerSecond);
        final double NormalAngularSpeed = TunerConstants.kAngularSpeedNormal.in(Units.RadiansPerSecond);
        final double FastAngularSpeed = TunerConstants.kAngularSpeedFast.in(Units.RadiansPerSecond);

        final SwerveRequest.FieldCentric fieldCentricDrive_withDeadband = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        
        final SwerveTelemetry logger = new SwerveTelemetry(MaxSpeed);

        Trigger leftTrigger = this.driverController.leftTrigger();
        Trigger rightTrigger = this.driverController.rightTrigger();

        // Drivetrain will execute this command periodically
        Drivetrain.setDefaultCommand(
            Drivetrain.applyRequest(() -> {
                boolean topSpeed = leftTrigger.getAsBoolean();
                boolean fineControl = rightTrigger.getAsBoolean();
                
                return fieldCentricDrive_withDeadband
                    // Drive forward with negative Y (forward)
                    .withVelocityX(
                        -driverController.getLeftY()
                        * (topSpeed ? MaxSpeed : LesserMaxSpeed)
                        * (fineControl ? ControllerConstants.FINE_CONTROL_MULT : 1)
                    )
                    // Drive left with negative X (left)
                    .withVelocityY(
                        -driverController.getLeftX()
                        * (topSpeed ? MaxSpeed : LesserMaxSpeed)
                        * (fineControl ? ControllerConstants.FINE_CONTROL_MULT : 1)
                    )
                    // Drive counterclockwise with negative X (left)
                    .withRotationalRate(
                        -driverController.getRightX()
                        * (topSpeed ? NormalAngularSpeed : FastAngularSpeed)
                        * (fineControl ? ControllerConstants.FINE_CONTROL_MULT : 1)
                    )
                    .withDeadband(ControllerConstants.DEADBAND * (fineControl ? LesserMaxSpeed : MaxSpeed))
                    .withRotationalDeadband(ControllerConstants.DEADBAND * (fineControl ? NormalAngularSpeed : FastAngularSpeed));
            }).ignoringDisable(true)
        );
        
        // Useful for testing
        // final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
        // this.driverController.y().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(
        //     new Rotation2d(
        //         Math.abs(driverController.getLeftY()) >= 0.25 ? -driverController.getLeftY() : 0,
        //         Math.abs(driverController.getLeftX()) >= 0.25 ? -driverController.getLeftX() : 0
        //     )
        // )));

        // POV / D-PAD
        final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        
        // This looks terrible, but I can't think of a better way to do it </3
        if (ControllerConstants.DPAD_DRIVE_INPUT) {
            /** POV angle : [X velocity, Y velocity] in m/s */
            final Map<Integer, Integer[]> povSpeeds = Map.ofEntries(
                Map.entry(  0, new Integer[]{ 1,  0}),
                Map.entry( 45, new Integer[]{ 1, -1}),
                Map.entry( 90, new Integer[]{ 0, -1}),
                Map.entry(135, new Integer[]{-1, -1}),
                Map.entry(180, new Integer[]{-1,  0}),
                Map.entry(225, new Integer[]{-1,  1}),
                Map.entry(270, new Integer[]{ 0,  1}),
                Map.entry(315, new Integer[]{ 1,  1})
            );

            povSpeeds.forEach(
                (Integer angle, Integer[] speeds) -> this.driverController.pov(angle).whileTrue(
                    Drivetrain.applyRequest(() -> {
                        boolean faster = leftTrigger.getAsBoolean();
                        boolean robotCentric = rightTrigger.getAsBoolean();
                        
                        return robotCentric
                            ? robotCentricDrive
                                .withVelocityX(speeds[0] * (faster ? 1.5 : 0.25))
                                .withVelocityY(speeds[1] * (faster ? 1.5 : 0.25))
                            : fieldCentricDrive
                                .withVelocityX(speeds[0] * (faster ? 1.5 : 0.25))
                                .withVelocityY(speeds[1] * (faster ? 1.5 : 0.25));
                    })
                )
            );
        }
        
        Drivetrain.registerTelemetry(logger::telemeterize);
    }

    /** Creates instances of each subsystem so periodic always runs. */
    private void initializeSubsystems() {
        LEDSubsystem.getInstance();
        VisionSubsystem.getInstance();
    }

    /** Register all NamedCommands for PathPlanner use */
    private void registerNamedCommands() {
        
    }

    /** Configures the button bindings of the driver controller. */
    public void configureDriverBindings() {
        this.driverController.b().onTrue(CommandGenerators.CancelAllCommands());

         /*
         * POV, joysticks, and start/back are all used in configureDrivetrain()
         *           Left joystick : Translational movement
         *          Right joystick : Rotational movement
         *    POV (overrides joys) : Directional movement -- 0.25 m/s
         */
        // Burger
        this.driverController.start().onTrue(CommandGenerators.SetForwardDirectionCommand());
        // Double Rectangle
        this.driverController.back().onTrue(CommandGenerators.ResetPoseUsingLimelightCommand());

        /*
         * Triggers are also used in configureDrivetrain()
         *      Left Trigger > 0.5 : Use TOP SPEED for joysticks
         *                           Use 1.5 m/s for POV
         *     Right Trigger > 0.5 : Use FINE CONTROL for joysticks
         *                           Use ROBOT CENTRIC for POV 
         */
        this.driverController.x().whileTrue(
            SwerveSubsystem.getInstance().applyRequest(() -> new SwerveRequest.SwerveDriveBrake())
        );

        this.driverController.leftBumper().whileTrue(new AlignToReefCommand(false));
        this.driverController.rightBumper().whileTrue(new AlignToReefCommand(true));
    }

    /** Configures the button bindings of the operator controller. */
    public void configureOperatorBindings() {
        // this.operatorController.b().onTrue(CommandGenerators.CancelAllCommands());
    }

    /**
     * Gets the instance of the driverController.
     * @return The driver controller.
     */
    public CommandXboxController getDriverController() {
        return this.driverController;
    }

    /**
     * Gets the instance of the operatorController.
     * @return The operator controller.
     */
    public CommandXboxController getOperatorController() {
        return this.operatorController;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return The command to run in autonomous.
     */
    public Command getAutonomousCommand() {
        return this.autoChooser.getSelected();
    }
}
