package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorS extends SubsystemBase {
    public class ElevatorConstants {
        public static final int LEADER_ID = 51;
        public static final int FOLLOWER_ID = 52;

        public static final double MOTOR_ROTATIONS_PER_METER = 1.0 / 5.0;

        public static final Distance MIN_EXTENSION = Inches.of(2.54);
        public static final Distance MAX_EXTENSION = Inches.of(6.995);
        public static final double MIN_LENGTH_ROTATIONS = MIN_EXTENSION.in(Meters)
                * ElevatorConstants.MOTOR_ROTATIONS_PER_METER;
        public static final double MAX_LENGTH_ROTATIONS = MAX_EXTENSION.in(Meters)
                * ElevatorConstants.MOTOR_ROTATIONS_PER_METER;

        public static final double KV = 2.0;
        public static final double KA = 2.5;
        public static final double KG = 1.0;

        private static TalonFXConfiguration configureLeader(TalonFXConfiguration config) {
            config.MotorOutput.withNeutralMode(NeutralModeValue.Brake).withInverted(InvertedValue.Clockwise_Positive);
            return config;
        }

        private static TalonFXConfiguration configureFollower(TalonFXConfiguration config) {
            config.MotorOutput.withNeutralMode(NeutralModeValue.Brake).withInverted(InvertedValue.Clockwise_Positive);
            return config;
        }

        public static final DCMotorSim m_motorSimModel = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(2), 0.001, MOTOR_ROTATIONS_PER_METER),
        DCMotor.getKrakenX60Foc(2));
    }

    private TalonFX leader = new TalonFX(ElevatorConstants.LEADER_ID);
    private TalonFX follower = new TalonFX(ElevatorConstants.FOLLOWER_ID);

    private StatusSignal<Angle> positionSignal = leader.getPosition();

    public final MechanismLigament2d elevatorVisualizer = new MechanismLigament2d("elevator",
            ElevatorConstants.MIN_EXTENSION.in(Meters), 90);

    public ElevatorS() {
        var config = new TalonFXConfiguration();
        leader.getConfigurator().refresh(config);
        leader.getConfigurator().apply(ElevatorConstants.configureLeader(config));

        var followerConfig = new TalonFXConfiguration();
        follower.getConfigurator().refresh(followerConfig);
        follower.getConfigurator().apply(ElevatorConstants.configureFollower(followerConfig));

        follower.setControl(new Follower(ElevatorConstants.LEADER_ID, false));

        if (RobotBase.isReal()) {
            leader.setPosition(ElevatorConstants.MIN_LENGTH_ROTATIONS);
        } else {
            leader.getSimState().setRawRotorPosition(ElevatorConstants.MIN_LENGTH_ROTATIONS);
        }
        setDefaultCommand(voltage(() -> 0.1));
    }

    VoltageOut voltage = new VoltageOut(0);

    public Command voltage(DoubleSupplier voltageSupplier) {
        return run(
                () -> {
                    leader.setControl(voltage.withOutput(voltageSupplier.getAsDouble()));
                });
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(positionSignal);
        elevatorVisualizer.setLength(getElevatorLengthMeters());

    }

    public void simulationPeriodic() {
        var talonFXSim = leader.getSimState();

        // set the supply voltage of the TalonFX
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // get the motor voltage of the TalonFX
        Voltage motorVoltage = talonFXSim.getMotorVoltageMeasure();

        // doesn't have any effect right now, don't know why
        talonFXSim.setRotorAcceleration(-ElevatorConstants.KG * ElevatorConstants.MOTOR_ROTATIONS_PER_METER);

        // use the motor voltage to calculate new position and velocity
        // using WPILib's DCMotorSim class for physics simulation
        ElevatorConstants.m_motorSimModel.setInputVoltage(motorVoltage.in(Volts));
        ElevatorConstants.m_motorSimModel.update(0.020); // assume 20 ms loop time

        // apply the new rotor position and velocity to the TalonFX;
        // note that this is rotor position/velocity (before gear ratio), but
        // DCMotorSim returns mechanism position/velocity (after gear ratio)
        
        talonFXSim.setRawRotorPosition(
            ElevatorConstants.m_motorSimModel.getAngularPosition()
                        .times(ElevatorConstants.MOTOR_ROTATIONS_PER_METER));
        talonFXSim.setRotorVelocity(
            ElevatorConstants.m_motorSimModel.getAngularVelocity()
                        .times(ElevatorConstants.MOTOR_ROTATIONS_PER_METER));
        
    }

    public double getElevatorLengthMeters() {
        double length = getMotorRotations() / ElevatorConstants.MOTOR_ROTATIONS_PER_METER;
        return length;
    }

    public double getMotorRotations() {
        return positionSignal.getValueAsDouble();
    }

}
