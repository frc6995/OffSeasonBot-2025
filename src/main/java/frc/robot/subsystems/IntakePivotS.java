package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.StatusSignal;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivotS extends SubsystemBase {

  public class IntakePivotConstants {

    public static final int INTAKE_PIVOT_MOTOR_CAN_ID = 40;
    public static final double INTAKE_PIVOT_UP_VOLTAGE = 2.0; // Voltage to move the intake pivot up
    public static final double INTAKE_PIVOT_DOWN_VOLTAGE = -2.0; // Voltage to move the intake pivot down

    public static final double FORWARD_SOFT_LIMIT = Units.degreesToRotations(40.0);
    public static final double REVERSE_SOFT_LIMIT = Units.degreesToRotations(-40.0);

  }

  private final TalonFX IntakePivotMotor = new TalonFX(IntakePivotConstants.INTAKE_PIVOT_MOTOR_CAN_ID);
  private final TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

  public IntakePivotS() {

    setMotorConfigs();
    IntakePivotMotor.getConfigurator().apply(motorConfigs);
  
    setDefaultCommand(stop());

  }

  private void setMotorConfigs() {
    motorConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakePivotConstants.FORWARD_SOFT_LIMIT;
    motorConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakePivotConstants.REVERSE_SOFT_LIMIT;
    motorConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
  }

  public Command voltage(double voltage) {
    return run(() -> IntakePivotMotor.setVoltage(voltage));
  }

  public Command slapDown() {
    return voltage(IntakePivotConstants.INTAKE_PIVOT_DOWN_VOLTAGE)
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
