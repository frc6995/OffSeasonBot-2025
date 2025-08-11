package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HandS extends SubsystemBase {
    // Define motors, sensors, and other components here
    // private final CANSparkMax handMotor = new CANSparkMax(Constants.HAND_MOTOR_ID, MotorType.kBrushless);
    // private final DigitalInput limitSwitch = new DigitalInput(Constants.LIMIT_SWITCH_ID);

    public HandS() {
        // Initialize motors and sensors
        // handMotor.setIdleMode(IdleMode.kBrake);
    }
    

    @Override
    public void periodic() {
        // Code to run periodically, such as checking sensors or updating motor states
    }

    // Define methods to control the hand system, e.g., open, close, check status, etc.
    
}
