// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.algae.AlgaeSubsystem;
import frc.robot.auto.PIDAlignCommand;
import frc.robot.constants.Constants.ControllerConstants;
import frc.robot.constants.Constants.ScoringConstants;
import frc.robot.constants.Constants.ShuffleboardTabNames;
import frc.robot.coral.AdjustCoralCommand;
import frc.robot.coral.CoralSubsystem;
import frc.robot.coral.IntakeCoralCommand;
import frc.robot.coral.OuttakeCoralCommand;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.elevator.MoveElevatorCommand;
import frc.robot.elevator.ZeroElevatorCommand;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.swerve.SwerveTelemetry;
import frc.robot.swerve.TunerConstants;
import frc.robot.led.LEDSubsystem;
import frc.robot.led.StatusColors;
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
    private final CommandXboxController driverController;
    private final CommandXboxController operatorController;
    private final XboxController driverController_HID;
    private final XboxController operatorController_HID;

    private Command auton = null;

    public RobotContainer() {
        this.driverController = new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_ID);
        this.operatorController = new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_ID);
        this.driverController_HID = this.driverController.getHID();
        this.operatorController_HID = this.operatorController.getHID();

        configureDrivetrain(); // This is done separately because it works differently from other Subsystems
        initializeSubsystems();

        // Register named commands for Pathplanner (always do this after subsystem initialization)
        registerNamedCommands();

        this.autoChooser = AutoBuilder.buildAutoChooser();
        this.autoChooser.onChange((Command autoCommand) -> {this.auton = autoCommand;}); // Default auto will be Commands.none()
        Shuffleboard.getTab(ShuffleboardTabNames.DEFAULT)
            .add("Auto Chooser", autoChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withSize(5, 2)
            .withPosition(1, 0);
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

        Supplier<Boolean> leftTrigger = () -> this.driverController_HID.getLeftTriggerAxis() >= 0.5;
        Supplier<Boolean> rightTrigger = () -> this.driverController_HID.getRightTriggerAxis() >= 0.5;

        // Drivetrain will execute this command periodically
        Drivetrain.setDefaultCommand(
            Drivetrain.applyRequest(() -> {
                boolean topSpeed = leftTrigger.get();
                boolean fineControl = rightTrigger.get();
                
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
                        boolean faster = leftTrigger.get();
                        boolean robotCentric = rightTrigger.get();
                        
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
        
        ElevatorSubsystem.getInstance();
        CoralSubsystem.getInstance();
        AlgaeSubsystem.getInstance();
    }

    /** Register all NamedCommands for PathPlanner use */
    private void registerNamedCommands() {
        NamedCommands.registerCommand("MoveElevatorToBottom",
            new MoveElevatorCommand(ScoringConstants.BOTTOM_HEIGHT, false, false));
        NamedCommands.registerCommand("MoveElevatorToL1Coral",
            new MoveElevatorCommand(ScoringConstants.L1_CORAL, false, false));
        NamedCommands.registerCommand("MoveElevatorToL2Coral",
            new MoveElevatorCommand(ScoringConstants.L2_CORAL, false, false));
        NamedCommands.registerCommand("MoveElevatorToL3Coral",
            new MoveElevatorCommand(ScoringConstants.L3_CORAL, false, false));
        NamedCommands.registerCommand("MoveElevatorToL2Algae",
            new MoveElevatorCommand(ScoringConstants.L2_ALGAE, false, false));
        
        // NamedCommands.registerCommand("MoveElevatorToBottom_Slow",
        //     new MoveElevatorCommand(ScoringConstants.BOTTOM_HEIGHT, true, false));
        // NamedCommands.registerCommand("MoveElevatorToL1Coral_Slow",
        //     new MoveElevatorCommand(ScoringConstants.L1_CORAL, true, false));
        // NamedCommands.registerCommand("MoveElevatorToL2Coral_Slow",
        //     new MoveElevatorCommand(ScoringConstants.L2_CORAL, true, false));
        // NamedCommands.registerCommand("MoveElevatorToL3Coral_Slow",
        //     new MoveElevatorCommand(ScoringConstants.L3_CORAL, true, false));
        // NamedCommands.registerCommand("MoveElevatorToL2Algae_Slow",
        //     new MoveElevatorCommand(ScoringConstants.L2_ALGAE, true, false));
        
        NamedCommands.registerCommand("IntakeCoral",
            new IntakeCoralCommand());
        NamedCommands.registerCommand("OuttakeCoral",
            new OuttakeCoralCommand());
        NamedCommands.registerCommand("AdjustCoral",
            new AdjustCoralCommand());
        NamedCommands.registerCommand("IntakeAlgaeAndHold",
            CommandGenerators.IntakeAlgaeAndHoldCommand());
        NamedCommands.registerCommand("OuttakeAlgaeAndStop",
            CommandGenerators.OuttakeAlgaeAndStopCommand());
        
        NamedCommands.registerCommand("PIDAlignRightReef",
            new PIDAlignCommand.Reef(1)
                .withTimeout(3)
        );
        NamedCommands.registerCommand("PIDAlignLeftReef",
            new PIDAlignCommand.Reef(-1)
                .withTimeout(3)
        );
        NamedCommands.registerCommand("PIDAlignCenterReef", // Algae
            new PIDAlignCommand.Reef(0)
                .withTimeout(3)
        );
        NamedCommands.registerCommand("PIDAlignProcessor",
            new PIDAlignCommand.Processor()
                .withTimeout(3)
        );
        
        NamedCommands.registerCommand("ReleaseAlgaeAndZeroElevator",
            CommandGenerators.InitialElevatorLiftAndZeroCommand());
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
        this.driverController.back().onTrue(CommandGenerators.ResetOdometryToOriginCommand());

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

        this.driverController.leftBumper().whileTrue(new PIDAlignCommand.Reef(-1));
        this.driverController.rightBumper().whileTrue(new PIDAlignCommand.Reef(1));
        this.driverController.a().whileTrue(new PIDAlignCommand.Reef(0));
        this.driverController.y().whileTrue(new PIDAlignCommand.Processor());
    }

    /** Configures the button bindings of the operator controller. */
    public void configureOperatorBindings() {
        this.operatorController.b().onTrue(CommandGenerators.CancelAllCommands());

        Supplier<Boolean> slowElevatorSupplier = () -> this.operatorController_HID.getRightTriggerAxis() >= 0.5;

        // Elevator
        this.operatorController.povDown()
            .toggleOnTrue(new MoveElevatorCommand(ScoringConstants.L1_CORAL, slowElevatorSupplier, true));
        this.operatorController.povRight()
            .toggleOnTrue(new MoveElevatorCommand(ScoringConstants.L2_CORAL, slowElevatorSupplier, true));
        this.operatorController.povUp()
            .toggleOnTrue(new MoveElevatorCommand(ScoringConstants.L3_CORAL, slowElevatorSupplier, true));
        this.operatorController.x()
            .toggleOnTrue(new MoveElevatorCommand(ScoringConstants.L2_ALGAE, slowElevatorSupplier, true));
        this.operatorController.leftTrigger()
            .onTrue(new MoveElevatorCommand(ScoringConstants.BOTTOM_HEIGHT, slowElevatorSupplier, false));
        this.operatorController.povLeft()
            .onTrue(new ZeroElevatorCommand());

        // Algae
        this.operatorController.a()
            .onTrue(AlgaeSubsystem.getInstance().runOnce(() -> AlgaeSubsystem.getInstance().intake()))
            .onFalse(AlgaeSubsystem.getInstance().runOnce(() -> AlgaeSubsystem.getInstance().hold()))
            .onFalse(Commands.runOnce(() -> LEDSubsystem.getInstance().setColor(StatusColors.OK)));
        this.operatorController.y()
            .onTrue(AlgaeSubsystem.getInstance().runOnce(() -> AlgaeSubsystem.getInstance().outtake()))
            .onFalse(AlgaeSubsystem.getInstance().runOnce(() -> AlgaeSubsystem.getInstance().stop()))
            .onFalse(Commands.runOnce(() -> LEDSubsystem.getInstance().setColor(StatusColors.OK)));

        // Coral
        this.operatorController.leftBumper().whileTrue(new IntakeCoralCommand());
        this.operatorController.rightBumper().whileTrue(new OuttakeCoralCommand());
        this.operatorController.rightStick().onTrue(new AdjustCoralCommand());
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
        if (this.auton == null) {
            this.auton = this.autoChooser.getSelected();
        }
        return this.auton;
    }
}
