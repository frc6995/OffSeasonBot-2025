package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusSignal;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivotS extends SubsystemBase {

  public class IntakePivotConstants {

    public static final int INTAKE_PIVOT_MOTOR_CAN_ID = 40;
    public static final double INTAKE_PIVOT_UP_VOLTAGE = 2.0; // Voltage to move the intake pivot up
    public static final double INTAKE_PIVOT_DOWN_VOLTAGE = -2.0; // Voltage to move the intake pivot down

    public static final Angle FORWARD_SOFT_LIMIT = Degrees.of(40.0);
    public static final Angle REVERSE_SOFT_LIMIT = Degrees.of(-40.0);

    public static final double MOTOR_ROTATIONS_PER_PIVOT_ROTATION = 12.5;

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

      return config;
    }

  }

  private final TalonFX IntakePivotMotor = new TalonFX(IntakePivotConstants.INTAKE_PIVOT_MOTOR_CAN_ID);

  public IntakePivotS() {

    var config = new TalonFXConfiguration();
    IntakePivotMotor.getConfigurator().refresh(config);
    IntakePivotMotor.getConfigurator().apply(IntakePivotConstants.configureMotor(config));

    setDefaultCommand(stop());

  }

  public Command voltage(double voltage) {
    return run(() -> IntakePivotMotor.setVoltage(voltage));
  }

  public Command slapDown() {
    return voltage(IntakePivotConstants.INTAKE_PIVOT_DOWN_VOLTAGE)
        .until(() -> IntakePivotMotor.getStatorCurrent().getValueAsDouble() > 50);
  }

  public Command slapUp() {
    return voltage(IntakePivotConstants.INTAKE_PIVOT_UP_VOLTAGE)
        .until(() -> IntakePivotMotor.getStatorCurrent().getValueAsDouble() > 50);
  }

  public Command stop() {
    return voltage(0);
  }

  @Override
  public void periodic() {
    // runs once every update cycle
  }

}
