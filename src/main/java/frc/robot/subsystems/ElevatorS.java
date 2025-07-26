package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class ElevatorS extends SubsystemBase {
    // Define motors, sensors, and other components here
    // private final CANSparkMax elevatorMotor = new CANSparkMax(Constants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
    // private final DigitalInput limitSwitch = new DigitalInput(Constants.LIMIT_SWITCH_ID);

    public ElevatorS() {
        // Initialize motors and sensors
        // elevatorMotor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        // Code to run periodically, such as checking sensors or updating motor states
    }

    // Define methods to control the elevator system, e.g., move up, move down, check status, etc.
    
}
