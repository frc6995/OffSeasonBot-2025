package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.wpilibj2.command.Commands.deadline;
import static edu.wpi.first.wpilibj2.command.Commands.defer;
import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.none;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.race;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.select;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.auto.AutoChooser;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ArmS;
import frc.robot.subsystems.ElevatorS;
import frc.robot.subsystems.HandS;
import frc.robot.subsystems.IntakePivotS;
import frc.robot.subsystems.IntakeRollerS;

public class Autos {
        private final AutoFactory m_factory;
        protected final CommandSwerveDrivetrain m_drivebase;
        protected final ArmS m_arm;
        protected final IntakePivotS m_intakepiv;
        protected final IntakeRollerS m_intakerol;
        protected final ElevatorS m_elev;
        protected final HandS m_hand;
        private final double SCORE_WAIT = 0.875;
    
        public Autos(CommandSwerveDrivetrain drivebase, ArmS arm, IntakePivotS intakepiv, IntakeRollerS intakerol, ElevatorS elev, HandS hand, AutoFactory factory) {
            m_drivebase = drivebase;
            m_arm = arm;
            m_intakepiv = intakepiv;
            m_intakerol = intakerol;
            m_elev = elev;
            m_hand = hand;
            m_factory = factory;
        }

        public void resetOdometry() {

        }
        public AutoRoutine FourCoralRight() {
            final AutoRoutine routine = m_factory.newRoutine("FourCoralRight");
            final AutoTrajectory traj = routine.trajectory("1");
        //toScoreJ.atTime(0).onTrue(m_elev.goToPosition(ElevatorS.Positions.HIGH_POSITION)).onTrue(m_arm.goToPosition(ArmS.Positions.CORAL_L4)).onTrue(m_hand.)
            final AutoTrajectory toScoreK = routine.trajectory("2");
            final AutoTrajectory toScoreL = routine.trajectory("3");
            final AutoTrajectory toScoreA = routine.trajectory("4");
           // traj.chain(toScoreK);
            //toScoreK.done().onTrue(waitSeconds(1.0).andThen(toScoreL.spawnCmd()));
            //toScoreK.chain(toScoreL);
            //toScoreL.done().onTrue(waitSeconds(1.0).andThen(toScoreA.spawnCmd()));
            //toScoreL.chain(toScoreA);
            routine.active().onTrue(
                traj.resetOdometry() 
                    .andThen(traj.cmd())
                    .andThen(Commands.waitSeconds(SCORE_WAIT))
                    .andThen(toScoreK.cmd())
                    .andThen(Commands.waitSeconds(SCORE_WAIT))
                    .andThen(toScoreL.cmd())
                    .andThen(Commands.waitSeconds(SCORE_WAIT))
                    .andThen(toScoreA.cmd())
            );
        return routine;
        }
        public AutoRoutine FourCoralLeft() {
            final AutoRoutine routine = m_factory.newRoutine("FourCoralLeft");
            final AutoTrajectory traj = routine.trajectory("5");
        //toScoreJ.atTime(0).onTrue(m_elev.goToPosition(ElevatorS.Positions.HIGH_POSITION)).onTrue(m_arm.goToPosition(ArmS.Positions.CORAL_L4)).onTrue(m_hand.)
            final AutoTrajectory toScoreK = routine.trajectory("6");
            final AutoTrajectory toScoreL = routine.trajectory("7");
            final AutoTrajectory toScoreA = routine.trajectory("8");
           // traj.chain(toScoreK);
            //toScoreK.done().onTrue(waitSeconds(1.0).andThen(toScoreL.spawnCmd())); Please work.
            //toScoreK.chain(toScoreL);
            //toScoreL.done().onTrue(waitSeconds(1.0).andThen(toScoreA.spawnCmd()));
            //toScoreL.chain(toScoreA);
            routine.active().onTrue(
                traj.resetOdometry() 
                    .andThen(traj.cmd())
                    .andThen(Commands.waitSeconds(SCORE_WAIT))
                    .andThen(toScoreK.cmd())
                    .andThen(Commands.waitSeconds(SCORE_WAIT))
                    .andThen(toScoreL.cmd())
                    .andThen(Commands.waitSeconds(SCORE_WAIT))
                    .andThen(toScoreA.cmd())
            );
        return routine;
        }
        /*public AutoRoutine BrokenThing() {
            final AutoRoutine routine = m_factory.newRoutine("BrokenThing");
            final AutoTrajectory traj = routine.trajectory("New Path");

            routine.active().onTrue(
                traj.resetOdometry().andThen(traj.cmd())
            );
        return routine;
        }*/
}
    /* public Autos(CommandSwerveDrivetrain drivebase, ArmS arm, IntakePivotS intakepiv, IntakeRollerS intakerol, ElevatorS elev, HandS hand, AutoFactory factory) {
        m_drivebase = drivebase;
        m_arm = arm;
        m_intakepiv = intakepiv;
        m_intakerol = intakerol;
        m_elev = elev;
        m_hand = hand;
        m_autoFactory = factory;

    }*/
