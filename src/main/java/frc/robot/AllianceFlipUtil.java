package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public class AllianceFlipUtil {
    public static double FIELD_WIDTH = Units.feetToMeters(26.0) + Units.inchesToMeters(5.0);
    public static double FIELD_LENGTH = Units.feetToMeters(57.0) + Units.inchesToMeters(6.875);

    public static boolean isRedAlliance() {
        return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }
    public static Translation2d flipTranslation(Translation2d t) {
        if (!isRedAlliance()) return t;
        return new Translation2d(FIELD_LENGTH - t.getX(), FIELD_WIDTH - t.getY());
    }

    public static Rotation2d flipRotation(Rotation2d r) {
        if (!isRedAlliance()) return r;
        return r.plus(Rotation2d.fromDegrees(180));
    }

    /** Flips a Pose2d to the red alliance */
    public static Pose2d flipPose(Pose2d p) {
        if (!isRedAlliance()) return p;
        return new Pose2d(flipTranslation(p.getTranslation()), flipRotation(p.getRotation()));
    }
}
