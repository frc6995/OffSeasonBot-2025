
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;


import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.generated.TunerConstants;
import yams.mechanisms.SmartMechanism;
import yams.motorcontrollers.remote.TalonFXWrapper;



public class YAMSIntakePivot extends SubsystemBase {

  public static final Angle SOME_ANGLE = Degrees.of(20);
  public static final Angle DOWN_ANGLE =  Degrees.of(-35);
  public static final Angle L1_ANGLE =  Degrees.of(65);
  public static final Angle HANDOFF_ANGLE = Degrees.of(135);

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)
  // Feedback Constants (PID Constants)
  .withClosedLoopController(18
  , 0, 0.2, DegreesPerSecond.of(458), DegreesPerSecondPerSecond.of(688))

  .withSimClosedLoopController(18, 0, 0.2, DegreesPerSecond.of(458), DegreesPerSecondPerSecond.of(688))
  // Feedforward Constants
  .withFeedforward(new ArmFeedforward(-0.1, 1.2, 0))
  .withSimFeedforward(new ArmFeedforward(0.0, 1.2, 0))
  // Telemetry name and verbosity level
  .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
  // Gearing from the motor rotor to final shaft.
  // In this example gearbox(3,4) is the same as gearbox("3:1","4:1") which corresponds to the gearbox attached to your motor.
  .withGearing(SmartMechanism.gearing(SmartMechanism.gearbox(12.5,1)))
  // Motor properties to prevent over currenting.
  .withMotorInverted(false)
  .withIdleMode(MotorMode.BRAKE)
  //.setMotionProfileMaxAcceleration(DegreesPerSecondPerSecond.of(300))
  
  .withStatorCurrentLimit(Amps.of(120)
  );

 void setMotionProfileMaxAcceleration(LinearAcceleration maxAcceleration) {
    // Set the max acceleration for motion profile
    //smcConfig.setMotionProfileMaxAcceleration(maxAcceleration);
  }

  // Vendor motor controller object
  private TalonFX Motor40 = new TalonFX(40, TunerConstants.kCANBus2);

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController IntakeSMC = new TalonFXWrapper(Motor40, DCMotor.getFalcon500(1), smcConfig);

  private final MechanismPositionConfig robotToMechanism = new MechanismPositionConfig()
      .withRelativePosition(new Translation3d(Meters.of(0.1), Meters.of(0), Meters.of(0.15)));


  private ArmConfig armCfg = new ArmConfig(IntakeSMC)
  // Soft limit is applied to the SmartMotorControllers PID

  .withHardLimit(Degrees.of(-25), Degrees.of(141))
  // Starting position is where your arm starts
  .withStartingPosition(Degrees.of(141))

  // Length and mass of your arm for sim.
  .withLength(Feet.of((14/12)))

  .withMOI(0.1055457256)


  
  // Telemetry name and verbosity for the arm.
  .withTelemetry("Arm", TelemetryVerbosity.HIGH)
  .withMechanismPositionConfig(robotToMechanism);


  // Arm Mechanism
  private Arm arm = new Arm(armCfg);

  
  /**
   * Set the angle of the arm.
   * @param angle Angle to go to.
   */
  public Command setAngle(Angle angle) {
 
    return arm.setAngle(angle);
    }

  /**
   * Move the arm up and down.
   * @param dutycycle [-1, 1] speed to set the arm too.
   */
 // public Command set(double dutycycle) { return arm.set(dutycycle);}

  /**
   * Run sysId on the {@link Arm}
   */
  public Command sysId() { return arm.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));}



  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    arm.updateTelemetry();
  
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    arm.simIterate();
  }
}