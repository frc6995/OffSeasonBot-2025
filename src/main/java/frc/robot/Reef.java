package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Reef {
        // 12 reef faces
    public static final Pose2d[] FACES = {
        ChoreoVariables.getPose("A"), //A
        ChoreoVariables.getPose("B"), //B
        ChoreoVariables.getPose("C"), //C
        ChoreoVariables.getPose("D"), //D
        ChoreoVariables.getPose("E"), //E
        ChoreoVariables.getPose("F"), //F
        ChoreoVariables.getPose("G"), //G
        ChoreoVariables.getPose("H"), //H
        ChoreoVariables.getPose("I"), //I
        ChoreoVariables.getPose("J"), //J
        ChoreoVariables.getPose("K"), //K
        ChoreoVariables.getPose("L"), //L

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
