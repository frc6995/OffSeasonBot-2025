// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmS;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HandS;


import frc.robot.subsystems.HandS.HandConstants;

import frc.robot.subsystems.YAMSIntakePivot;
import frc.robot.subsystems.YAMSIntakeRollerS;

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

    public final YAMSIntakeRollerS intakeRoller = new YAMSIntakeRollerS();

    public final HandS handRoller = new HandS();

    public final ArmS Arm = new ArmS();

    public final YAMSIntakePivot yIntakePivot = new YAMSIntakePivot();

    private Mechanism2d VISUALIZER;

    public RobotContainer() {
        VISUALIZER = logger.MECH_VISUALIZER; 

        configureBindings();
        SmartDashboard.putData("Visualizer", VISUALIZER);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                                   // negative Y
                                                                                                   // (forward)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                    // negative X (left)
                ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));


                //set button bindings
                joystick.a().onTrue(intakeCoral());
                joystick.b().onTrue(Handoff());
               joystick.x().onTrue(Stow());
                joystick.y().whileTrue(L1Score());


                //joystick.leftTrigger().whileTrue(Arm_L2scoring());
                //joystick.rightTrigger().whileTrue(Arm_L3Scoring());

                //Hand Off sequence
                //joystick.rightBumper().onTrue(Commands.sequence(Commands.parallel(Arm_Hand_Off_Angle(), intakeCoral()).withTimeout(0.5)
                //,Commands.parallel(L1Score(), Hand_Rollers_In())
                //));
                //Scoring sequence 
                //joystick.leftBumper().onTrue(Commands.sequence(Stow(), L1Score()));
                

                joystick.rightBumper().onTrue(Arm.setAngle(Degrees.of(120)));
                joystick.leftBumper().onTrue(Arm.setAngle(Degrees.of(0)));



        
                drivetrain.registerTelemetry(logger::telemeterize);

            }
        
            public Command getAutonomousCommand() {
                return Commands.print("No autonomous command configured");
        
            }
            

            //Commands combining multiple subsystem functions
            public Command intakeCoral() {
                return Commands.race(yIntakePivot.setAngle(yIntakePivot.DOWN_ANGLE), intakeRoller.coralIntake());
            }
        
            public Command Stow() {
                return yIntakePivot.setAngle(yIntakePivot.L1_ANGLE);
            }
            public Command L1Score() {
                return intakeRoller.outTakeRollers();
            }
        

            public Command Handoff() {
                return yIntakePivot.setAngle(yIntakePivot.HANDOFF_ANGLE);
            }
            /*
            public Command Arm_L2scoring(){
                return Arm.moveToAngle(PivotConstants.SCORE_ANGLE_L2);
            }
            public Command Arm_L3Scoring(){
                return Arm.moveToAngle(PivotConstants.SCORE_ANGLE_L3);
            }
            public Command Arm_L4Scoring(){
                return Arm.moveToAngle(PivotConstants.SCORE_ANGLE_L4);
            }
            public Command Arm_Hand_Off_Angle(){
                return Arm.moveToAngle(PivotConstants.HANDOFF_ANGLE);
            }
            public Command Hand_Voltage_Scoring(){
                return handRoller.setHandRollerVoltage(HandConstants.HAND_ROLLER_OUT_VOLTAGE);
            }
            public Command Hand_Rollers_In(){
                return handRoller.HandCoralIntake();
            }
            public Command Arm_Scoring_postion(){
                return Arm.moveToAngle(PivotConstants.ARM_SOME_ANGLE);
            }*/
        }

