package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;

public class IntakePivotS extends SubsystemBase {

  //Primary constants:
  public class IntakePivotConstants {

    public static final int INTAKE_PIVOT_MOTOR_CAN_ID = 40;
    public static final double INTAKE_PIVOT_UP_VOLTAGE = -2.5; // Voltage to move the intake pivot up
    public static final double INTAKE_PIVOT_DOWN_VOLTAGE = 2.5; // Voltage to move the intake pivot down

    public static final Angle FORWARD_SOFT_LIMIT = Degrees.of(40000.0);
    public static final Angle REVERSE_SOFT_LIMIT = Degrees.of(-40000.0);
    public static final Angle SOME_ANGLE = Degrees.of(20);

    public static final double MOTOR_ROTATIONS_PER_PIVOT_ROTATION = 12.5;
    public static final double kArmP = 0.5; // Talon FX PID P gain (tune this)
    public static final double kArmI = 0.0; // Talon FX PID I gain (tune this)
    public static final double kArmD = 0.0; // Talon FX PID D gain (tune this)
    public static final double kArmS = 0.1; // Feedforward Static gain (tune this)
    public static final double kArmG = 2; // Feedforward Gravity gain (tune this)
    public static final double kArmV = 1.0; // Feedforward Velocity gain (tune this)
    public static final double kArmA = 0.0; // Feedforward Acceleration gain (tune this)
    public static final double kArmMaxVoltage = 12.0; // Maximum voltage for the arm motor

    public static final double kArmOffset = Math.toRadians(-84.5);
    // Constants for the Kraken motor encoder
    public static final double kEncoderTicksPerRevolution = 2048.0; // Kraken X60 built-in encoder resolution
    public static final double kSensorToMechanismRatio = 12.5; // Gear ratio from encoder to arm mechanism
    public static final double kArmGearRatio = kSensorToMechanismRatio; // For clarity, same as above
    public static final double kArmPositionToleranceRotations = 0.01; // Tolerance for position control in rotations
 
    private static final ArmFeedforward intakeFeedforward = new ArmFeedforward(
        kArmS, kArmG, kArmV, kArmA);
    public static final PositionTorqueCurrentFOC positionRequest = new PositionTorqueCurrentFOC(0); // Initialize with a target

    private static TalonFXConfiguration configureMotor(TalonFXConfiguration config) {
      config.MotorOutput.withNeutralMode(NeutralModeValue.Coast)
          .withInverted(InvertedValue.CounterClockwise_Positive);
      config.SoftwareLimitSwitch
          .withForwardSoftLimitEnable(true)
          .withForwardSoftLimitThreshold(FORWARD_SOFT_LIMIT)
          .withReverseSoftLimitEnable(true)
          .withReverseSoftLimitThreshold(REVERSE_SOFT_LIMIT);

      config.CurrentLimits.withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(Amps.of(50));
      config.Feedback.withSensorToMechanismRatio(MOTOR_ROTATIONS_PER_PIVOT_ROTATION);
      config.Slot0 = new Slot0Configs();
      config.Slot0.kP = kArmP;
      config.Slot0.kI = kArmI;
      config.Slot0.kD = kArmD;

      return config;
    }
  }

  //Other constants
  public double targetAngle = 0.0;

  double feedforwardVoltage;

  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  private final TalonFX IntakePivotMotor = new TalonFX(IntakePivotConstants.INTAKE_PIVOT_MOTOR_CAN_ID,
      TunerConstants.kCANBus2);

public final MechanismLigament2d IntakePivotVisualizer = new MechanismLigament2d("Intake", 1, 0); 

  //set initiall configurations/values
  public IntakePivotS() {

    var config = new TalonFXConfiguration();

    IntakePivotMotor.getConfigurator().refresh(config);
    IntakePivotMotor.getConfigurator().apply(IntakePivotConstants.configureMotor(config));
    IntakePivotMotor.setPosition(0.1);

    setDefaultCommand(hold());

    SignalLogger.start();

  }

  //Commands:

  public Command hold() {
    return Commands.sequence(runOnce(() -> targetAngle = getArmAngleRadians()),
        run(() -> {
          IntakePivotMotor.setControl(
              IntakePivotConstants.positionRequest.withPosition(targetAngle).withFeedForward(feedforwardVoltage));

        }));
  }

  public Command moveToAngle(Angle someAngle) {
    return run(() -> {
      setArmTargetAngle(targetAngle);
    });
  }

  //Periodic:
  @Override
  public void periodic() {

    //SignalLogger.writeDouble("Intake/TargetAngle", targetAngle);
    SmartDashboard.putNumber("Intake/TargetAngle", targetAngle);

    

    // Get the current arm angle from the encoder (Kraken's position is in rotations)
    double encoderRotations = IntakePivotMotor.getRotorPosition().getValueAsDouble();

    // Convert encoder rotations to radians for the mechanism
    // (encoder_rotations / ratio) * (2 * pi)
    double armAngleRadians = (encoderRotations / IntakePivotConstants.kArmGearRatio) * 2 * Math.PI
        + IntakePivotConstants.kArmOffset;

    IntakePivotVisualizer.setAngle(new Rotation2d(Radians.of(armAngleRadians)));

    // Calculate the feedforward voltage for gravity compensation
    // The current arm angle is used for feedforward, as it's the most accurate representation
    // The TalonFX handles velocity and acceleration internally with its PID or motion profiling

    feedforwardVoltage = IntakePivotConstants.intakeFeedforward.calculate(
        armAngleRadians, 0.0, 0.0); // Desired velocity and acceleration are 0 for gravity comp

    // Update the arbitrary feedforward in the control request
    IntakePivotConstants.positionRequest.withFeedForward(feedforwardVoltage / IntakePivotConstants.kArmMaxVoltage); // Scale to [-1, 1]

    // Set the control request with the target position and feedforward
    // Target position is in rotations, so convert from targetAngle (radians)
    double targetRotations = (targetAngle - IntakePivotConstants.kArmOffset) / (2 * Math.PI)
        * IntakePivotConstants.kArmGearRatio;
    IntakePivotConstants.positionRequest.withPosition(targetRotations);
    IntakePivotMotor.setControl(IntakePivotConstants.positionRequest);
  }

  //Methods:
  public void setArmTargetAngle(double angleRadians) {
    // Convert the target angle (radians) to TalonFX position units (rotations)
    targetAngle = angleRadians;
  }

  public double getArmAngleRadians() {
    return (IntakePivotMotor.getRotorPosition().getValueAsDouble() / IntakePivotConstants.kArmGearRatio) * 2 * Math.PI
        + IntakePivotConstants.kArmOffset;
  }

  public boolean isAtTarget() {
    // Since onboard PID is used, it is necessary to check if the error is within a
    // tolerance
    // The tolerance value might need adjustment based on the mechanism
    double currentPositionRotations = IntakePivotMotor.getRotorPosition().getValueAsDouble();

    double targetPositionRotations = (targetAngle - IntakePivotConstants.kArmOffset) / (2 * Math.PI)
        * IntakePivotConstants.kArmGearRatio;
    return Math
        .abs(currentPositionRotations - targetPositionRotations) < IntakePivotConstants.kArmPositionToleranceRotations;

  }
}
