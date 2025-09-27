package frc.robot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import com.therekrab.autopilot.Autopilot.APResult;



import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAlign extends Command{
    private final APTarget m_target;
    private final CommandSwerveDrivetrain m_drivetrain;
    private static final APConstraints kConstraints = new APConstraints()
    .withAcceleration(5.0)
    .withJerk(2.0);

private static final APProfile kProfile = new APProfile(kConstraints)
    .withErrorXY(Centimeters.of(2))
    .withErrorTheta(Degrees.of(0.5))
    .withBeelineRadius(Centimeters.of(8));

public static final Autopilot kAutopilot = new Autopilot(kProfile);
    private final SwerveRequest.FieldCentricFacingAngle m_request = new SwerveRequest.FieldCentricFacingAngle()
        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
        .withDriveRequestType(DriveRequestType.Velocity)
        .withHeadingPID(4, 0, 0); /* tune this for your robot! */
  
  
    public AutoAlign(APTarget target, CommandSwerveDrivetrain drivetrain) {
      m_target = target;
      m_drivetrain = drivetrain;
    }

    public void execute() {
      ChassisSpeeds robotRelativeSpeeds = m_drivetrain.getState().Speeds;
      Pose2d pose = m_drivetrain.getState().Pose;
  
      APResult out = kAutopilot.calculate(pose, robotRelativeSpeeds, m_target);
  
      m_drivetrain.setControl(m_request
          .withVelocityX(out.vx())
          .withVelocityY(out.vy())
          .withTargetDirection(out.targetAngle()));
    }

    public boolean isFinished() {
      return kAutopilot.atTarget(m_drivetrain.getState().Pose, m_target);
    }
    /* public enum ReefSide {
    https://github.dev/frc6995/Robot-2025/blob/7d05a7edd5c5a2bfa4c94d6e8bca32373e7bf839/src/main/java/frc/robot/Autos.java#L838
    
        R1(POI.A, POI.B, POI.R1, AlgaeHeight.HIGH, Rotation2d.kZero),
        R2(POI.C, POI.D, POI.R2, AlgaeHeight.LOW, Rotation2d.fromDegrees(60)),
        R3(POI.E, POI.F, POI.R3, AlgaeHeight.HIGH, Rotation2d.fromDegrees(120)),
        R4(POI.G, POI.H, POI.R4, AlgaeHeight.LOW, Rotation2d.k180deg),
        R5(POI.I, POI.J, POI.R5, AlgaeHeight.HIGH, Rotation2d.fromDegrees(240)),
        R6(POI.K, POI.L, POI.R6, AlgaeHeight.LOW, Rotation2d.fromDegrees(300));

        public final POI left;
        public final POI right;
        public final POI algae;
        public final AlgaeHeight algaeHeight;

        public final Rotation2d batteryFaceAlgaeBlueHeading;
        public final Rotation2d pivotFaceAlgaeBlueHeading;
        public final Supplier<Rotation2d> batteryFaceAlgaeFlippedHeading;
        public final Supplier<Rotation2d> pivotFaceAlgaeFlippedHeading;
        private ReefSide(POI left, POI right, POI algae, AlgaeHeight height, Rotation2d batteryFaceAlgaeBlueHeading) {
        this.left = left;
        this.right = right;
        this.algae = algae;
        this.algaeHeight = height;
        this.batteryFaceAlgaeBlueHeading = batteryFaceAlgaeBlueHeading;
        this.pivotFaceAlgaeBlueHeading = AllianceFlipUtil.flip(batteryFaceAlgaeBlueHeading);
        this.batteryFaceAlgaeFlippedHeading = AllianceFlipUtil.getFlipped(batteryFaceAlgaeBlueHeading);
        this.pivotFaceAlgaeFlippedHeading = AllianceFlipUtil.getFlipped(pivotFaceAlgaeBlueHeading);
        }

    }

    public ReefSide closestSide() {
        var reef = POI.REEF.flippedPose();
        var pose = m_drivebase.getPose();
        var direction = pose.relativeTo(reef).getTranslation().getAngle().getRadians();
        if (direction < -5 * Math.PI / 6 || direction > 5 * Math.PI / 6) {
        return ReefSide.R1;
        }
        if (direction >= -5 * Math.PI / 6 && direction < -3 * Math.PI / 6) {
        return ReefSide.R2;
        }
        if (direction >= -3 * Math.PI / 6 && direction < -1 * Math.PI / 6) {
        return ReefSide.R3;
        }
        if (direction >= -1 * Math.PI / 6 && direction < 1 * Math.PI / 6) {
        return ReefSide.R4;
        }
        if (direction >= 1 * Math.PI / 6 && direction < 3 * Math.PI / 6) {
        return ReefSide.R5;
        } else {
        return ReefSide.R6;
        }
    }*/

  }
  