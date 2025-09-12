package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;


import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
public class ArmS extends SubsystemBase {
    // Define motors, sensors, and other components here
    public ArmS() {
        // Initialize motors and sensors
        var config = new TalonFXConfiguration();

        ARM_MOTOR.getConfigurator().refresh(config);
        ARM_MOTOR.getConfigurator().apply(PivotConstants.configureMotor(config));
    
        SignalLogger.start();
    }
    public class PivotConstants {

        public static final int ARM_MOTOR_CAN_ID = 60;
        //Pivot angles
        //public static final double ARM_SOME_ANGLE = 20;
        public static final double SCORE_ANGLE_L2 = 45;
        public static final double SCORE_ANGLE_L3 = 55;
        public static final double SCORE_ANGLE_L4 = 65;
        public static final double HANDOFF_ANGLE = -90;
        public static final double ARM_SOME_ANGLE = 130;
        //Constants used for PID and Feedforward
        public static final double MOTOR_ROTATIONS_PER_ARM_PIVOT_ROTATION = 12.5;//idk what it is
        public static final double ArmP = 6; // Talon FX PID P gain (tune this) 6
        public static final double ArmI = 0; // Talon FX PID I gain (tune this)
        public static final double ArmD = 0.7; // Talon FX PID D gain (tune this) 0.4
        public static final double ArmS = -0.05; // Feedforward Static gain (tune this) -0.05
        public static final double ArmG = 0.9; // Feedforward Gravity gain (tune this)
        public static final double ArmV = 0; // Feedforward Velocity gain (tune this)
        public static final double ArmA = 0; // Feedforward Acceleration gain (tune this)
    
        //Pivot Offset from Zero degrees (when the code starts, it always resets the angle to zero so this is neccesary
        // for offseting it to the upper hard stop)
        public static final double ArmOffSet = Math.toRadians(-90);
        // Constants for the Kraken motor encoder
        public static final double ARMSensorToMechanismRatio = 12.5; // Gear ratio from encoder to arm mechanism
        public static final double ArmGearRatio = ARMSensorToMechanismRatio; // For clarity, same as above
     
        //Initiallizes the feedforward controller to a variable
        private static final ArmFeedforward ARMINTAKEFEEDFOWARD = new ArmFeedforward(
            ArmS, ArmG, ArmV, ArmA);
       
        //Motor configuration
        private static TalonFXConfiguration configureMotor(TalonFXConfiguration config) {
          config.MotorOutput.withNeutralMode(NeutralModeValue.Brake)
              .withInverted(InvertedValue.Clockwise_Positive);
    
          config.CurrentLimits.withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(Amps.of(50));
          config.Feedback.withSensorToMechanismRatio(MOTOR_ROTATIONS_PER_ARM_PIVOT_ROTATION);
          config.Slot0 = new Slot0Configs();
    
          return config;
        }
      }
    
  //Other constants

  //Sets initial target angle to hard stop
  public static double targetAngle = 170;

  //Initiallizes the PID controller to a variable
  private static PIDController m_pidController = 
  new PIDController(
      PivotConstants.ArmP,
      PivotConstants.ArmI,
      PivotConstants.ArmD

  );

  //Sets up visualization 
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  //Sets the motor to the CAN ID and CAN Bus
  private static final TalonFX ARM_MOTOR = new TalonFX(PivotConstants.ARM_MOTOR_CAN_ID,
      TunerConstants.kCANBus2);

  //Sets up a simulation visualizer for our mechanism
//public final MechanismLigament2d armVisualizer = new MechanismLigament2d("Pivot", 1, 0); 

  //set initiall configurations/values


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
    SmartDashboard.putNumber("ArmTargetAngle", targetAngle);
    SmartDashboard.putNumber("ArmIntake/currentAngleRadians", getArmAngleRadians());
    SmartDashboard.putNumber("Armsupplycurrent", ARM_MOTOR.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Armstatorcurrent", ARM_MOTOR.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Arm volage", ARM_MOTOR.getMotorVoltage().getValueAsDouble());

    //sets initiall angle for simulation
    //armVisualizer.setAngle(new Rotation2d(Degrees.of(getArmAngleRadians() * 180/Math.PI)));

    //Sets the voltage of the motor to the sum of Feedforward and PID controllers
      ARM_MOTOR.setVoltage(PivotConstants.ARMINTAKEFEEDFOWARD.calculate(
      getArmAngleRadians(), 1, 1) +
    m_pidController.calculate(getArmAngleRadians(), (targetAngle * (Math.PI/180))));
  }

  //Methods:
  //Gets the output of the motor sensor, then converts it to the accurate radian measure for the pivot
  public double getArmAngleRadians() {
    return (ARM_MOTOR.getRotorPosition().getValueAsDouble() / PivotConstants.ArmGearRatio) * 2 * Math.PI + PivotConstants.ArmOffSet;
  }



}
