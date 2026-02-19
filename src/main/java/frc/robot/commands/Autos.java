package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import dev.doglog.DogLog;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.Swerve;

/** All auton routines. */
public class Autos {
  private final AutoFactory _factory;

  private final Swerve _swerve;

  private SendableChooser<Side> _sideSelector = new SendableChooser<Side>();

  // auton objectives
  private enum Side {
    LEFT("Left "),
    CENTER("Center "),
    RIGHT("Right ");

    private final String _dir;

    private Side(String dir) {
      _dir = dir;
    }

    public String getDirectory() {
      return _dir;
    }
  }

  private final BooleanSubscriber _shootPreload = DogLog.tunable("Shoot Preload", false);
  private final BooleanSubscriber _bump = DogLog.tunable("Bump", false);
  private final BooleanSubscriber _climb = DogLog.tunable("Climb", false);

  public Autos(Swerve swerve) {
    _swerve = swerve;

    _factory =
        new AutoFactory(
            _swerve::getPose,
            _swerve::resetPose,
            _swerve::followTrajectory,
            true,
            _swerve,
            (traj, isActive) -> {
              DogLog.log("Auto/Current Trajectory", traj.getPoses());
              DogLog.log("Auto/Current Trajectory Name", traj.name());
              DogLog.log("Auto/Current Trajectory Duration", traj.getTotalTime());
              DogLog.log("Auto/Current Trajectory Is Active", isActive);
            });
  }

  public AutoRoutine example() {
    AutoRoutine routine = _factory.newRoutine("example");

    AutoTrajectory exampleTraj =
        routine.trajectory(_sideSelector.getSelected().getDirectory() + "example");

    routine.active().onTrue(sequence(exampleTraj.resetOdometry(), exampleTraj.cmd()));

    return routine;
  }
}
