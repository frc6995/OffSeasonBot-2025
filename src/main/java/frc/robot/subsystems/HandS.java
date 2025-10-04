package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HandS extends SubsystemBase {

    public class HandConstants {

        public static final int HAND_ROLLER_MOTOR_CAN_ID = 61;
        public static final double HAND_ROLLER_IN_VOLTAGE = -6; // Voltage to move the intake rollers in
        public static final double HAND_ROLLER_OUT_VOLTAGE = 2.7;

    }

    // Configure addtional motor variables
    private final TalonFX /* Change-> */ HandRollersMotor = new TalonFX(HandConstants.HAND_ROLLER_MOTOR_CAN_ID,
            TunerConstants.kCANBus2);

    public HandS() {

        // Set to new variable name and configure addtional motors if necessary
        HandRollersMotor.getConfigurator().apply(new TalonFXConfiguration());
        HandRollersMotor.getConfigurator().apply(new MotorOutputConfigs());

    }

    // Code from IntakeRollerS.java: change motor variable names
    public Command setHandRollerVoltage(double voltage) {
        return run(() -> HandRollersMotor.setVoltage(voltage));
    }

    // voltage commands:
    public Command handRollersStart() {
        return setHandRollerVoltage(HandConstants.HAND_ROLLER_IN_VOLTAGE)
                .withTimeout(0.15);

    }

    public Command handRollersUntilStop() {
        return setHandRollerVoltage(HandConstants.HAND_ROLLER_IN_VOLTAGE)
                .until(() -> HandRollersMotor.getStatorCurrent().getValueAsDouble() > 50);
    }

    public Command HandCoralIntake() {
        return Commands.sequence(handRollersStart(), handRollersUntilStop());
    }

    public Command handOutTakeRollers() {
        return setHandRollerVoltage(HandConstants.HAND_ROLLER_OUT_VOLTAGE);
    }

    public Command handStopRollers() {
        return run(() -> HandRollersMotor.setVoltage(0)); // Set voltage to 0 to stop the rollers
    }

    public Command handjectL1Coral() {
        return setHandRollerVoltage(HandConstants.HAND_ROLLER_OUT_VOLTAGE);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hand voltage", HandRollersMotor.getMotorVoltage().getValueAsDouble());
        // Code to run periodically, such as checking sensors or updating motor states
    }

    // Define methods to control the hand system, e.g., open, close, check status,
    // etc.

}
