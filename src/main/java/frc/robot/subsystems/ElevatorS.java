package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorS extends SubsystemBase {
    public class ElevatorConstants {
        public static final int LEADER_ID = 51;
        public static final int FOLLOWER_ID = 52;

        public static final Distance FIRT_STAGE_LENGTH = Inches.of(6.995);
        public static final Distance SECOND_STAGE_LENGTH = Inches.of(2.54);
        public static final double MOTOR_ROTATIONS_PER_METER = 1.0 / 5.0;

        public static final Distance MIN_EXTENSION = Inches.of(2.54);
        public static final Distance MAX_EXTENSION = Inches.of(6.995);
        public static final double MIN_LENGTH_ROTATIONS = MIN_EXTENSION.in(Meters)
                * ElevatorConstants.MOTOR_ROTATIONS_PER_METER;
        public static final double MAX_LENGTH_ROTATIONS = MAX_EXTENSION.in(Meters)
                * ElevatorConstants.MOTOR_ROTATIONS_PER_METER;

        public static final double KV = 2.0;
        public static final double KA = 2.5;

        private static TalonFXConfiguration configureLeader(TalonFXConfiguration config) {
            config.MotorOutput.withNeutralMode(NeutralModeValue.Brake).withInverted(InvertedValue.Clockwise_Positive);
            return config;
        }

        private static TalonFXConfiguration configureFollower(TalonFXConfiguration config) {
            config.MotorOutput.withNeutralMode(NeutralModeValue.Brake).withInverted(InvertedValue.Clockwise_Positive);
            return config;
        }

        public static ElevatorSim sim = new ElevatorSim(KV, KA, DCMotor.getKrakenX60(2), MIN_EXTENSION.in(Meters),
                MAX_EXTENSION.in(Meters), false, MIN_EXTENSION.in(Meters));
    }

    private TalonFX leader = new TalonFX(ElevatorConstants.LEADER_ID);
    private TalonFX follower = new TalonFX(ElevatorConstants.FOLLOWER_ID);

    private StatusSignal<Angle> positionSignal = leader.getPosition();

    public final MechanismLigament2d elevatorVisualizer = new MechanismLigament2d("elevator",
            ElevatorConstants.MIN_EXTENSION.in(Meters), -90);

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
            ElevatorConstants.sim.setState(VecBuilder.fill(ElevatorConstants.MIN_EXTENSION.in(Meters), 90.0));
        }
        setDefaultCommand(voltage(() -> 0.5));
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
        // Code to run periodically, such as checking sensors or updating motor states
        BaseStatusSignal.refreshAll(positionSignal);
        elevatorVisualizer.setLength(getElevatorLengthMeters());

    }

    public void simulationPeriodic() {
        var simState = leader.getSimState();
        simState.setSupplyVoltage(12);
        // simState.getMotorVoltage is counterclockwise negative
        ElevatorConstants.sim.update(0.02);
        var rotorPos = ElevatorConstants.sim.getPositionMeters() * ElevatorConstants.MOTOR_ROTATIONS_PER_METER;
        var rotorVel = ElevatorConstants.sim.getVelocityMetersPerSecond() * ElevatorConstants.MOTOR_ROTATIONS_PER_METER;
        simState.setRawRotorPosition(rotorPos);
        simState.setRotorVelocity(rotorVel);
      }

    public double getElevatorLengthMeters() {
        double length = positionSignal.getValueAsDouble() / ElevatorConstants.MOTOR_ROTATIONS_PER_METER;
        return length;
    }

}
