package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public class IntakeRollerS extends SubsystemBase {

    public class IntakeRollersConstants {

        // Subsystem constants
        public static final int INTAKE_ROLLER_MOTOR_CAN_ID = 4;
        public static final double INTAKE_ROLLER_IN_VOLTAGE = -6; // Voltage to move the intake rollers in
        public static final double INTAKE_ROLLER_OUT_VOLTAGE = 2.7; // Voltage to move the intake rollers out

    }

    // Configure motor variable
    private final TalonFX intakeRollersMotor = new TalonFX(IntakeRollersConstants.INTAKE_ROLLER_MOTOR_CAN_ID,
            TunerConstants.kCANBus2);

    public MechanismLigament2d intakeRollerVisualizer = new MechanismLigament2d("Intake Roller", 0.5, 0);

    public IntakeRollerS() {
        // Initialize motor configuration
        intakeRollersMotor.getConfigurator().apply(new TalonFXConfiguration());
        intakeRollersMotor.getConfigurator().apply(new MotorOutputConfigs() {
        });

        // Set a default motor command
        setDefaultCommand(stopRollers());
    }

    // Commands

    // Sets up constructor for simpler voltage command code; see below how its
    // implemented
    public Command setRollerVoltage(double voltage) {
        return run(() -> intakeRollersMotor.setVoltage(voltage));
    }

    // voltage commands:
    public Command intakeRollersStart() {
        return setRollerVoltage(IntakeRollersConstants.INTAKE_ROLLER_IN_VOLTAGE)
                .withTimeout(0.15);

    }

    public Command intakeRollersUntilStop() {
        return setRollerVoltage(IntakeRollersConstants.INTAKE_ROLLER_IN_VOLTAGE)
                .until(() -> intakeRollersMotor.getStatorCurrent().getValueAsDouble() > 50);
    }

    public Command coralIntake() {
        return Commands.sequence(intakeRollersStart(), intakeRollersUntilStop());
    }

    public Command outTakeRollers() {
        return setRollerVoltage(IntakeRollersConstants.INTAKE_ROLLER_OUT_VOLTAGE);
    }

    public Command stopRollers() {
        return run(() -> intakeRollersMotor.setVoltage(0)); // Set voltage to 0 to stop the rollers
    }

    public Command ejectL1Coral() {
        return setRollerVoltage(IntakeRollersConstants.INTAKE_ROLLER_OUT_VOLTAGE);
    }

    // Add Smart dash board
    @Override
    public void periodic() {
        intakeRollerVisualizer
                .setAngle(new Rotation2d(Rotations.of(intakeRollersMotor.getRotorPosition().getValueAsDouble())));
    }
}
