// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import frc.robot.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakePivotS;
import frc.robot.subsystems.IntakeRollerS;


public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public static final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final IntakePivotS intakePivot = new IntakePivotS();

    public final IntakeRollerS intakeRoller = new IntakeRollerS();

    private final AutoFactory autoFactory;
    private Autos autoRoutines;
    private Mechanism2d VISUALIZER; 
    SendableChooser<Command> m_chooser = new SendableChooser<>();
 
    public RobotContainer() {
        
        VISUALIZER = logger.MECH_VISUALIZER; 
        logger.addIntake(intakePivot.IntakePivotVisualizer);
        configureBindings();
        SmartDashboard.putData("Visualzer", VISUALIZER);
        

        autoFactory = drivetrain.createAutoFactory();
        Autos autoRoutines = new Autos(drivetrain, null, intakePivot, intakeRoller, null, null, autoFactory);
        m_chooser.setDefaultOption("FourCoralRight", autoRoutines.FourCoralRight());
        SmartDashboard.putData(m_chooser); 

    }
    

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        joystick.a().onTrue(intakeRoller.coralIntake());
        joystick.x().whileTrue(intakePivot.slapDown());
        joystick.y().whileTrue(intakePivot.slapUp());
        joystick.b().whileTrue(intakeRoller.outTakeRollers());
        /*
         * joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
         * l1-score-and-intake-merge
         * joystick.b().whileTrue(drivetrain.applyRequest(() ->
         * point.withModuleDirection(new Rotation2d(-joystick.getLeftY(),
         * -joystick.getLeftX()))
         * ));
         * 
         * // Run SysId routines when holding back/start and X/Y.
         * // Note that each routine should be run exactly once in a single log.
         * joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction
         * .kForward));
         * joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction
         * .kReverse));
         * joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(
         * Direction.kForward));
         * joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(
         * Direction.kReverse));
         * 
         * // reset the field-centric heading on left bumper press
         * joystick.leftBumper().onTrue(drivetrain.runOnce(() ->
         * drivetrain.seedFieldCentric()));
         * 
         * drivetrain.registerTelemetry(logger::telemeterize);
         * 
         * joystick.x().whileTrue(intakePivot.slapDown());
         * 
         * joystick.y()
         * .whileTrue(intakeRoller.intakeRollers()) // Start rollers while the button is
         * pressed
         * .onFalse(intakeRoller.stopRollers()); // Stop rollers when the button is
         * released/*
         */
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command L1Score() {
        return Commands.sequence(intakePivot.dropTillStall(), intakeRoller.ejectL1Coral());
    }
    
    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
        
    }

    /*
     * public Command Intake() {
     * return Commands.parallel(intakePivot.slapDown(),intakeRoller.intakeRollers())
     * .until(()->intakeRoller.getCurrent() > 10).andThen(intakePivot.slapUp());
     * }
     * 
     * public Command IntakeRollersStartCommand() {
     * return(
     * new ScheduleCommand(intakeRoller.intakeRollersStart()));
     * 
     * }
     */


}
