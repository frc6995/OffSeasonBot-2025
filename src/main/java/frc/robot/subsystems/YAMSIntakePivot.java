package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.positional.Elevator;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static yams.mechanisms.SmartMechanism.gearbox;
import static yams.mechanisms.SmartMechanism.gearing;



public class YAMSIntakePivot extends SubsystemBase {
    private final TalonFX intakeMotor = new TalonFX (43, TunerConstants.kCANBus2);
    private final SmartMotorController motor = new TalonFXWrapper(intakeMotor, DCMotor.getNEO(1),
    new SmartMotorControllerConfig(this)
          .withClosedLoopController(4.0, 0.0, 0.0)
          .withSoftLimit(Degrees.of(-30), Degrees.of(100))
          .withGearing(gearing(gearbox(12.5)))
          .withIdleMode(MotorMode.BRAKE)
          .withTelemetry("YAMSIntake", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(120))
          .withFeedforward(new ArmFeedforward(0, 1, 0, 0))
          .withControlMode(ControlMode.CLOSED_LOOP));

    private final Arm YAMSIntakePivot = new Arm(new ArmConfig(motor)
      .withLength(Meters.of(0.135))
      .withStartingPosition(Degrees.of(135))
      .withTelemetry("ArmExample", TelemetryVerbosity.HIGH)
  );

  @Override
  public void periodic() {
    YAMSIntakePivot.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    YAMSIntakePivot.simIterate();
  }

  public Command setAngle(Angle angle) {
    return YAMSIntakePivot.setAngle(angle);
  }
}
