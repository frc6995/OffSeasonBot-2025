package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.StatusSignal;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivotS extends SubsystemBase {

    public class IntakePivotConstants {

       public static final int INTAKE_PIVOT_MOTOR_CAN_ID = 40; 
       public static final double INTAKE_PIVOT_UP_VOLTAGE = 0.5; // Voltage to move the intake pivot up
       public static final double INTAKE_PIVOT_DOWN_VOLTAGE = -0.5; // Voltage to move the intake pivot down
        
    }
    // Define motors, sensors, and other components here
    // private final CANSparkMax intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    // private final DigitalInput limitSwitch = new DigitalInput(Constants.LIMIT_SWITCH_ID);

    private final TalonFX inPivMotor = new TalonFX(IntakePivotConstants.INTAKE_PIVOT_MOTOR_CAN_ID);


    public IntakePivotS() {
        
        inPivMotor.getConfigurator().apply(new TalonFXConfiguration());
        
        // Initialize motors and sensors
        // intakeMotor.setIdleMode(IdleMode.kBrake);
        
    }

  public Command voltage(double voltage) {
    return this.run(() -> inPivMotor.setVoltage(voltage));
 }

   public Command slapDown() {
    return voltage(IntakePivotConstants.INTAKE_PIVOT_DOWN_VOLTAGE).until(() -> inPivMotor.getStatorCurrent().getValueAsDouble() > 50);
   }

    @Override
    public void periodic() {
        // Code to run periodically, such as checking sensors or updating motor states
    }

    // Define methods to control the intake system, e.g., start, stop, check status, etc.
    
}

