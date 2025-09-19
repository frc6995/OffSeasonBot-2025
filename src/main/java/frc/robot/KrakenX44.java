package frc.robot;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class KrakenX44 {
      public static DCMotor getX44(int numMotors) {
    return new DCMotor(
        12, 4.05, 275, 1.4, Units.rotationsPerMinuteToRadiansPerSecond(7530), numMotors);
  }
}
