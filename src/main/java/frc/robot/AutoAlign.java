package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.run;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAlign {

    private Pose2d targetPose;

    private Pose2d currentPose = new Pose2d();

    private CommandSwerveDrivetrain drivetrain;

    //private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

    public AutoAlign(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public void UpdateCurrentPose(Pose2d currentPose) {
        this.currentPose = currentPose;
    }

    public Command AlignToPose(Pose2d targetPose) {
        double xError = targetPose.getX() - currentPose.getX();
        double yError = targetPose.getY() - currentPose.getY();
        double headingError = targetPose.getRotation().getDegrees() - currentPose.getRotation().getDegrees();

        // return run(() -> drivetrain.applyRequest(
        //         () -> drive.withVelocityX(xError).withVelocityY(yError).withRotationalRate(headingError)));
        
        return Commands.none();
    }
}
