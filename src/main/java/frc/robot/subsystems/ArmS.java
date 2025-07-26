package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmS extends SubsystemBase {
    // Define motors, sensors, and other components here
    // private final CANSparkMax armMotor = new CANSparkMax(Constants.ARM_MOTOR_ID, MotorType.kBrushless);
    // private final DigitalInput limitSwitch = new DigitalInput(Constants.LIMIT_SWITCH_ID);

    public ArmS() {
        // Initialize motors and sensors
        // armMotor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        // Code to run periodically, such as checking sensors or updating motor states
    }

    // Define methods to control the arm system, e.g., move up, move down, check status, etc.
    
}
