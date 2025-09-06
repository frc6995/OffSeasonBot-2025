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
import edu.wpi.first.math.controller.PIDController;
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

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;

public class IntakePivotS extends SubsystemBase {

  //Primary constants:
  public class IntakePivotConstants {

    public static final int INTAKE_PIVOT_MOTOR_CAN_ID = 40;

    //Pivot angles
    public static final double SOME_ANGLE = 20;
    public static final double DOWN_ANGLE = -23;
    public static final double L1_ANGLE = 65;
    public static final double HANDOFF_ANGLE = 135;

    //Constants used for PID and Feedforward
    public static final double MOTOR_ROTATIONS_PER_PIVOT_ROTATION = 12.5;

    //Pivot Offset from Zero degrees (when the code starts, it always resets the angle to zero so this is neccesary
    // for offseting it to the upper hard stop)
    public static final double kArmOffset = Math.toRadians(137);
    // Constants for the Kraken motor encoder
    public static final double kSensorToMechanismRatio = 12.5; // Gear ratio from encoder to arm mechanism
    public static final double kArmGearRatio = kSensorToMechanismRatio; // For clarity, same as above
 
    //Initiallizes the feedforward controller to a variable

   
    //Motor configuration
    private static TalonFXConfiguration configureMotor(TalonFXConfiguration config) {
      config.MotorOutput.withNeutralMode(NeutralModeValue.Brake)
          .withInverted(InvertedValue.Clockwise_Positive);

      config.CurrentLimits.withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(Amps.of(50));
      config.Feedback.withSensorToMechanismRatio(MOTOR_ROTATIONS_PER_PIVOT_ROTATION);
      config.Slot0 = new Slot0Configs();
      var slot0Configs = config.Slot0;
           // slot0Configs.kG = getArmFeedforward(); //Gravity compensation
                        slot0Configs.kS = 0.05; // Add 0.25 V output to overcome static friction
                        slot0Configs.kV = 0; // A velocity target of 1 rps results in 0.12 V output
                        slot0Configs.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
                        slot0Configs.kP = 0.1; // A position error of 2.5 rotations results in 12 V output
                        slot0Configs.kI = 0; // no output for integrated error
                        slot0Configs.kD = 0; // A velocity error of 1 rps results in 0.1 V output
                  
                        var motionMagicConfigs = config.MotionMagic;
                        motionMagicConfigs.MotionMagicCruiseVelocity = 0; // Target cruise velocity of 80 rps
                        motionMagicConfigs.MotionMagicAcceleration = 0; // Target acceleration of 160 rps/s (0.5 seconds)
                        motionMagicConfigs.MotionMagicJerk = 0; // Target jerk of 1600 rps/s/s (0.1 seconds)
                  
                        return config;
                      }
                    }
                  
                    //Other constants
                  
                    //Sets initial target angle to hard stop
                    public static double targetAngle = 141;
                  
                    //Initiallizes the PID controller to a variable
                  
                  
                    //Sets up visualization 
                    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
                  
                    //Sets the motor to the CAN ID and CAN Bus
                    private static final TalonFX IntakePivotMotor = new TalonFX(IntakePivotConstants.INTAKE_PIVOT_MOTOR_CAN_ID,
                        TunerConstants.kCANBus2);
                  
                    //Sets up a simulation visualizer for our mechanism
                  public final MechanismLigament2d IntakePivotVisualizer = new MechanismLigament2d("Intake", 1, 0); 
                  
                    //set initiall configurations/values
                    public IntakePivotS() {
                  
                      var config = new TalonFXConfiguration();
                  
                      IntakePivotMotor.getConfigurator().refresh(config);
                      IntakePivotMotor.getConfigurator().apply(IntakePivotConstants.configureMotor(config));
                  
                      SignalLogger.start();
                    }
                  
                    //Commands:
                  
                    //Takes in an angle, and sets the PID target angle to that angle
                    public Command moveToAngle(double someAngle) {
                      return run(() -> {
                        targetAngle = someAngle;
                     
                      });
                    }
                  
                    //Periodic (always active):
                    @Override
                    public void periodic() {
                  
                      //Puts values to Smart Dashboard. Add as needed for simulation
                      SmartDashboard.putNumber("Intake/TargetAngle", targetAngle);
                      SmartDashboard.putNumber("Intake/currentAngleRadians", getArmAngleRadians());
                      SmartDashboard.putNumber("supplycurrent", IntakePivotMotor.getSupplyCurrent().getValueAsDouble());
                      SmartDashboard.putNumber("statorcurrent", IntakePivotMotor.getStatorCurrent().getValueAsDouble());
                      SmartDashboard.putNumber("volage", IntakePivotMotor.getMotorVoltage().getValueAsDouble());
                  
                      //sets initiall angle for simulation
                      IntakePivotVisualizer.setAngle(new Rotation2d(Degrees.of(getArmAngleRadians() * 180/Math.PI)));
                  
                      // create a Motion Magic request, voltage output
                      final MotionMagicVoltage m_request = new MotionMagicVoltage(0).withFeedForward(getArmFeedforward());
                  
                  // set target position to 100 rotations
                      IntakePivotMotor.setControl(m_request.withPosition(-15));
                    }
                  
                    //Methods:
                    //Gets the output of the motor sensor, then converts it to the accurate radian measure for the pivot
                    public double getArmAngleRadians() {
                return (IntakePivotMotor.getRotorPosition().getValueAsDouble() / IntakePivotConstants.kArmGearRatio) * 2 * Math.PI + IntakePivotConstants.kArmOffset;
              }
            
              public double getArmFeedforward() {
            return 0.95 * Math.cos(IntakePivotMotor.getRotorPosition().getValueAsDouble() / IntakePivotConstants.kArmGearRatio * 2 * Math.PI + IntakePivotConstants.kArmOffset);
  }

}
