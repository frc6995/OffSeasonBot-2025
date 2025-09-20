package frc.robot;

//import frc.robot.LimelightHelpers;
//import frc.robot.LimelightHelpers.PoseEstimate;


import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ArmS;
import frc.robot.subsystems.ElevatorS;
import frc.robot.subsystems.HandS;
import frc.robot.subsystems.IntakePivotS;
import frc.robot.subsystems.IntakeRollerS;
public class AutoAlign {
    private CommandSwerveDrivetrain drivetrain;
    private final Supplier<Pose2d> limelightPoseSupplier;
    private final Pose2d targetPose;

    public AutoAlign(CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> limelightPoseSupplier, Pose2d targetPose) {
        this.drivetrain = drivetrain;
        this.limelightPoseSupplier = limelightPoseSupplier;
        this.targetPose = targetPose;

    }

    public void align() {
        Pose2d current = limelightPoseSupplier.get();
        if (current == null) return;

        double xError = targetPose.getX() - current.getX();
        double yError = targetPose.getY() - current.getY();
        double headingError = targetPose.getRotation().getDegrees() - current.getRotation().getDegrees();
        
        

    }
    /**
     * Pose supplier initialized
     * get the pose
     * to find the error between target and current pose for x y and heading(degrees/rotations)
     * move to that
     */
}
