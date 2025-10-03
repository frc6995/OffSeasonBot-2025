package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Reef {
        // 12 reef faces
    public static final Pose2d[] FACES = {
        new Pose2d(3.02, 4.19, Rotation2d.fromDegrees(270)), //A
        new Pose2d(3.02, 3.87, Rotation2d.fromDegrees(270)), //B
        new Pose2d(3.62, 2.84, Rotation2d.fromDegrees(330)), //C
        new Pose2d(3.90, 2.68, Rotation2d.fromDegrees(330)), //D
        new Pose2d(5.08, 2.68, Rotation2d.fromDegrees(30)),  //E
        new Pose2d(5.36, 2.84, Rotation2d.fromDegrees(30)),  //F
        new Pose2d(5.96, 3.87, Rotation2d.fromDegrees(90)),  //G
        new Pose2d(5.96, 4.19, Rotation2d.fromDegrees(90)),  //H
        new Pose2d(5.36, 5.21, Rotation2d.fromDegrees(150)), //I
        new Pose2d(5.08, 5.37, Rotation2d.fromDegrees(150)), //J
        new Pose2d(3.90, 5.37, Rotation2d.fromDegrees(210)), //K
        new Pose2d(3.62, 5.21, Rotation2d.fromDegrees(210))  //L
    };
    public enum ReefSide {
        R1(FACES[0], "left"),
        R2(FACES[1], "right"),
        R3(FACES[2], "left"),
        R4(FACES[3], "right"),
        R5(FACES[4], "right"),
        R6(FACES[5], "left"),
        R7(FACES[6], "right"),
        R8(FACES[7], "left"),
        R9(FACES[8], "right"),
        R10(FACES[9], "left"),
        R11(FACES[10], "left"), 
        R12(FACES[11], "right");

        public final Pose2d pose;
        public final String side; // "left" or "right"

        ReefSide(Pose2d pose, String side) {
            this.pose = pose;
            this.side = side;
        }
    }

    public static ReefSide closestSide(Pose2d robotPose) {
        ReefSide closest = ReefSide.R1;
        double minDistance = Double.MAX_VALUE;
        for (ReefSide side : ReefSide.values()) {
            double distance = robotPose.getTranslation().getDistance(side.pose.getTranslation());
            if (distance < minDistance) {
                minDistance = distance;
                closest = side;
            }
        }
        return closest;
    }
    public static ReefSide closestSide(Pose2d robotPose, String branch) {
        ReefSide closest = null;
        double minDistance = Double.MAX_VALUE;
        for (ReefSide side : ReefSide.values()) {
            if (!side.side.equalsIgnoreCase(branch)) continue;
            double distance = robotPose.getTranslation().getDistance(side.pose.getTranslation());
            if (distance < minDistance) {
                minDistance = distance;
                closest = side;
            }
        }
        return closest;
    }
}
