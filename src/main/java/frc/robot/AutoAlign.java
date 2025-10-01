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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAlign extends Command{
    private APTarget m_target;
    private final CommandSwerveDrivetrain m_drivetrain;
    private static final APConstraints kConstraints = new APConstraints()
    .withAcceleration(5.0)
    .withJerk(2.0);
    private String branch;

    private static final APProfile kProfile = new APProfile(kConstraints)
        .withErrorXY(Centimeters.of(2))
        .withErrorTheta(Degrees.of(0.5))
        .withBeelineRadius(Centimeters.of(8));

    public static final Autopilot kAutopilot = new Autopilot(kProfile);


    private final SwerveRequest.FieldCentricFacingAngle m_request = new SwerveRequest.FieldCentricFacingAngle()
        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
        .withDriveRequestType(DriveRequestType.Velocity)
        .withHeadingPID(4, 0, 0); /* tune this for your robot! */
        
        
        
    public AutoAlign(APTarget target, CommandSwerveDrivetrain drivetrain, String branch) {
          m_target = target;
          m_drivetrain = drivetrain;
          this.branch = branch;
    }

    public void execute() {
      Pose2d robotPose = m_drivetrain.getState().Pose;
      ChassisSpeeds robotSpeeds = m_drivetrain.getState().Speeds;


      Pose2d flippedPose = AllianceFlipUtil.flipPose(robotPose);

      // Find closest reef side for the chosen branch
      Reef.ReefSide closest = Reef.closestSide(flippedPose, branch);

      Pose2d targetPose = AllianceFlipUtil.flipPose(closest.pose);

      m_target = new APTarget(targetPose);

      APResult out = kAutopilot.calculate(robotPose, robotSpeeds, m_target);

      m_drivetrain.setControl(m_request
              .withVelocityX(out.vx())
              .withVelocityY(out.vy())
              .withTargetDirection(out.targetAngle()));
    }


    public boolean isFinished() {
      return kAutopilot.atTarget(m_drivetrain.getState().Pose, m_target);
    }


}
