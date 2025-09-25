package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import yams.mechanisms.SmartMechanism;
import yams.mechanisms.config.ShooterConfig;
import yams.mechanisms.velocity.Shooter;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class YAMSIntakeRollerS extends SubsystemBase {

    public static class YAMSIntakeRollerConstants {
        public static final int ROLLER_CAN_ID = 41;

        public static final AngularVelocity OUTTAKE_VELOCITY = RPM.of(70);
        public static final AngularVelocity INTAKE_VELOCITY = RPM.of(-70);
        public static final Voltage INTAKE_VOLTAGE = Volts.of(-6);
        public static final Voltage OUTTAKE_VOLTAGE = Volts.of(6);

        public static final Current STATOR_LIMIT = Amps.of(50);
    }

    private SmartMotorControllerConfig rollerMotorConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            // PID
            .withClosedLoopController(30, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
            .withSimClosedLoopController(30, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
            // Feedforward
            .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
            .withSimFeedforward(new SimpleMotorFeedforward(0, 0.3, 0.1))
            // telemetry/simulation
            .withTelemetry("IntakeRollerMotor", TelemetryVerbosity.HIGH)
            // gear ratio between motor and final shaft
            .withGearing(SmartMechanism.gearing(SmartMechanism.gearbox(0.4)))
            // motor properties
            .withMotorInverted(false)
            .withIdleMode(MotorMode.COAST)
            .withStatorCurrentLimit(YAMSIntakeRollerConstants.STATOR_LIMIT)
            .withClosedLoopRampRate(Seconds.of(0.25));

    // TalonFX motor controller object
    private TalonFX rollerMotor = new TalonFX(YAMSIntakeRollerConstants.ROLLER_CAN_ID, TunerConstants.kCANBus2);

    // create SmartMotorController with the TalonFX
    private SmartMotorController talonFXSmartMotorController = new TalonFXWrapper(
            rollerMotor, DCMotor.getKrakenX60(1), rollerMotorConfig);

    // config object for the rollers, using a ShooterConfig object but not
    // specifying all the values that don't matter for a basic roller
    private ShooterConfig rollerConfig = new ShooterConfig(talonFXSmartMotorController)
            .withDiameter(Inches.of(2))
            .withMass(Pounds.of(0.6))
            .withTelemetry("IntakeRollerMech", TelemetryVerbosity.HIGH);

    // using a Shooter object for our roller, since the mechanisms function
    // similarly
    private Shooter roller = new Shooter(rollerConfig);

    public YAMSIntakeRollerS() {
        setDefaultCommand(set(0));
    }

    /**
     * Gets the current velocity of the shooter.
     *
     * @return Shooter velocity.
     */
    public AngularVelocity getVelocity() {
        return roller.getSpeed();
    }

    /**
     * Set the shooter velocity.
     *
     * @param speed Speed to set.
     * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
     */
    public Command setVelocity(AngularVelocity speed) {
        return roller.setSpeed(speed);
    }

    public Command setVoltage(Voltage volts) {
        return run(() -> rollerMotor.setVoltage(volts.in(Volts)));
    }

    /**
     * Set the dutycycle of the shooter.
     *
     * @param dutyCycle DutyCycle to set.
     * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
     */
    public Command set(double dutyCycle) {
        return roller.set(dutyCycle);
    }

    public Command intakeRollersStart() {
        return setVoltage(YAMSIntakeRollerConstants.INTAKE_VOLTAGE)
                .withTimeout(0.15);

    }

    public Command intakeRollersUntilStop() {
        return setVoltage(YAMSIntakeRollerConstants.INTAKE_VOLTAGE)
                .until(() -> rollerMotor.getStatorCurrent().getValueAsDouble() > YAMSIntakeRollerConstants.STATOR_LIMIT
                        .in(Amps));
    }

    public Command outTakeRollers() {
        return setVoltage(YAMSIntakeRollerConstants.OUTTAKE_VOLTAGE);
    }

    public Command coralIntake() {
        return Commands.sequence(intakeRollersStart(), intakeRollersUntilStop());
    }

    @Override
    public void periodic() {
        roller.updateTelemetry();

    }

    @Override
    public void simulationPeriodic() {
        roller.simIterate();
    }

}
