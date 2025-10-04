package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static yams.mechanisms.SmartMechanism.gearbox;
import static yams.mechanisms.SmartMechanism.gearing;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import yams.mechanisms.SmartMechanism;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;
public class ElevatorS extends SubsystemBase {
    // Define motors, sensors, and other components here
    // private final CANSparkMax elevatorMotor = new CANSparkMax(Constants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
    // private final DigitalInput limitSwitch = new DigitalInput(Constants.LIMIT_SWITCH_ID);
;
  public final Distance kElevatorMinHeight = Inches.of(13);
  public final Distance kElevatorMaxHeight = Inches.of(76);
    
    private SmartMotorControllerConfig smcElevConfig = new SmartMotorControllerConfig(this)
    .withFollowers(Pair.of(new TalonFX(52, TunerConstants.kCANBus), false))
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withMechanismCircumference((Meters.of(Inches.of(0.25).in(Meters) *16)))
    .withClosedLoopController(10, 0, 0.2, MetersPerSecond.of(1), MetersPerSecondPerSecond.of(5))
    .withSimClosedLoopController(2, 0, 0, MetersPerSecond.of(2), MetersPerSecondPerSecond.of(5))
//    .withSoftLimit(Inches.of(0), Inches.of(77.5))
      .withGearing(gearing(gearbox(1, 5)))
//      .withExternalEncoder(armMotor.getAbsoluteEncoder())
      .withIdleMode(MotorMode.BRAKE)
      .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
//      .withSpecificTelemetry("ElevatorMotor", motorTelemetryConfig)
      .withStatorCurrentLimit(Amps.of(120))
//      .withVoltageCompensation(Volts.of(12))
      .withMotorInverted(false)
//      .withClosedLoopRampRate(Seconds.of(0.25))
//      .withOpenLoopRampRate(Seconds.of(0.25))
      //.withFeedforward(new ElevatorFeedforward(0, 2.28, 3.07, 0.41));
      .withFeedforward(new ElevatorFeedforward(0, 0.35, 0.65,0.01 )); //KG: gravity compensation, KV: velicity (multiplied by closed loop controller velocity)
      //KA: acceleration (multiplied by closed loop controller acceleration). Tune Velocity until line is near same angle as target, tune accel until curve mathces accuately. 
      // After this, tune KP to help with any small inconsitencies.
    
    private TalonFX leadMotor = new TalonFX(51, TunerConstants.kCANBus);

    private SmartMotorController elevatorLeadSMC = new TalonFXWrapper(leadMotor, DCMotor.getFalcon500(2), smcElevConfig);
    private final MechanismPositionConfig robotToMechanism = new MechanismPositionConfig().withRelativePosition(new Translation3d(Meters.of(0.1016),Meters.of(0),Meters.of(0))); 
    
    private ElevatorConfig elevconfig = new ElevatorConfig(elevatorLeadSMC)
    .withStartingHeight(Inches.of(13))
    .withHardLimits(Inches.of(13), Inches.of(76))
    .withSoftLimits(Inches.of(13),  Inches.of(76))
    .withTelemetry("Elevator", TelemetryVerbosity.HIGH)
    .withMass(Pounds.of(14))
    .withMechanismPositionConfig(robotToMechanism);
    
    private Elevator elevator = new Elevator(elevconfig);

    public ElevatorS() {
        // Initialize motors and sensors
        // elevatorMotor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        // Code to run periodically, such as checking sensors or updating motor states
        elevator.updateTelemetry();
    }
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        elevator.simIterate();
    }
  /**
   * Example command factory method.
   *
   * @return a command
   */
    public Command setHeight(Distance height) {
        return elevator.setHeight(height);}

    public Command set(double dutycycle) {
        return elevator.set(dutycycle);}
        

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public Command sysId() {
    // Query some boolean state, such as a digital sensor.
    return elevator.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
}

}
  


 

