// Robot.java or similar (robot-wide constants)
import com.ctre.phoenix6.configs.Slot0Configs;

public static final double kArmP = 0.5; // Talon FX PID P gain (tune this)
public static final double kArmI = 0.0; // Talon FX PID I gain (tune this)
public static final double kArmD = 0.0; // Talon FX PID D gain (tune this)

public static final double kArmS = 0.1; // Feedforward Static gain (tune this)
public static final double kArmG = 0.75; // Feedforward Gravity gain (tune this)
public static final double kArmV = 1.0; // Feedforward Velocity gain (tune this)
public static final double kArmA = 0.0; // Feedforward Acceleration gain (tune this)

public static final double kArmOffset = Math.toRadians(90.0); // Offset to measure angle from horizontal

// Constants for the Kraken motor encoder
public static final double kEncoderTicksPerRevolution = 2048.0; // Kraken X60 built-in encoder resolution
public static final double kSensorToMechanismRatio = 12.5; // Gear ratio from encoder to arm mechanism
public static final double kArmGearRatio = kSensorToMechanismRatio; // For clarity, same as above

// ArmSubsystem.java
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC; // Import for onboard position control
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private final TalonFX armMotor = new TalonFX(Constants.kArmMotorPort); // Replace with your CAN ID
    private final ArmFeedforward armFeedforward = new ArmFeedforward(
            Constants.kArmS, Constants.kArmG, Constants.kArmV, Constants.kArmA);

    // Control request object for onboard position control with arbitrary feedforward
    private final PositionTorqueCurrentFOC positionRequest = new PositionTorqueCurrent(0) // Initialize with a target
            .withEnableFOC(true) // Enable Field-Oriented Control (recommended)
            .withArbitraryFeedForward(0.0); // Will be updated dynamically

    public ArmSubsystem() {
        // Configure Kraken motor and encoder
        armMotor.setInverted(false); // Adjust based on your arm direction
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Configure onboard PID gains (Slot 0 for position control)
        config.Slot0 = new Slot0Configs();
        config.Slot0.kP = Constants.kArmP;
        config.Slot0.kI = Constants.kArmI;
        config.Slot0.kD = Constants.kArmD;
        // You might need to set kF if you're using velocity control, but for position
        // with arbitrary feedforward, it's typically handled by the feedforward term.

        armMotor.getConfigurator().apply(config);

        // Set the encoder position to 0 (or a known initial value)
        armMotor.getRotorPosition().set(0.0);
    }

    @Override
    public void periodic() {
        // Get the current arm angle from the encoder (Kraken's position is in rotations)
        double encoderRotations = armMotor.getRotorPosition().getValue();

        // Convert encoder rotations to radians for the mechanism
        // (encoder_rotations / ratio) * (2 * pi)
        double armAngleRadians = (encoderRotations / Constants.kArmGearRatio) * 2 * Math.PI + Constants.kArmOffset;

        // Calculate the feedforward voltage for gravity compensation
        // The current arm angle is used for feedforward, as it's the most accurate representation
        // The TalonFX handles velocity and acceleration internally with its PID or motion profiling
        double feedforwardVoltage = armFeedforward.calculate(
                armAngleRadians, 0.0, 0.0); // Desired velocity and acceleration are 0 for gravity comp

        // Update the arbitrary feedforward in the control request
        positionRequest.withArbitraryFeedForward(feedforwardVoltage / Constants.kArmMaxVoltage); // Scale to [-1, 1]

        // Set the control request with the target position and feedforward
        // Target position is in rotations, so convert from targetAngle (radians)
        double targetRotations = (targetAngle - Constants.kArmOffset) / (2 * Math.PI) * Constants.kArmGearRatio;
        positionRequest.withPosition(targetRotations);
        armMotor.setControl(positionRequest);
    }

    public void setArmTargetAngle(double angleRadians) {
        // Convert the target angle (radians) to TalonFX position units (rotations)
        targetAngle = angleRadians;
    }

    public double getArmAngleRadians() {
        return (armMotor.getRotorPosition().getValue() / Constants.kArmGearRatio) * 2 * Math.PI + Constants.kArmOffset;
    }

    public boolean isAtTarget() {
        // Since onboard PID is used, it is necessary to check if the error is within a tolerance
        // The tolerance value might need adjustment based on the mechanism
        double currentPositionRotations = armMotor.getRotorPosition().getValue();
        double targetPositionRotations = (targetAngle - Constants.kArmOffset) / (2 * Math.PI) * Constants.kArmGearRatio;
        return Math.abs(currentPositionRotations - targetPositionRotations) < Constants.kArmPositionToleranceRotations;
    }
}

// ArmCommands.java (example command)
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetArmAngle extends CommandBase {
    private final ArmSubsystem arm;
    private final double targetAngle;

    public SetArmAngle(ArmSubsystem arm, double targetAngle) {
        this.arm = arm;
        this.targetAngle = targetAngle;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setArmTargetAngle(targetAngle);
    }

    @Override
    public void execute() {
        // ArmSubsystem's periodic() handles the actual control
    }

    @Override
    public boolean isFinished() {
        return arm.isAtTarget();
    }

    @Override
    public void end(boolean interrupted) {
        // When the command ends, the arm will hold its last commanded position
    }
