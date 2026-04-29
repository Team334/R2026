package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.autonomous;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;
import dev.doglog.DogLog;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.InputStream;
import frc.lib.fault.FaultLogger;
import frc.lib.fault.FaultsTable.FaultType;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.intake.IntakeFeed;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.utils.FieldUtil;
import frc.robot.utils.ShotParameters;
import java.io.File;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.Supplier;

/** Generates the auto command. */
public class Auto {
  // choreo
  private final AutoFactory _factory;
  private final HashMap<String, Supplier<Command>> _bindings = new HashMap<>();

  private Command _autoCmd;

  private boolean _aimAtTarget = false;

  // files
  private final String layoutDir = "layouts";
  private final ObjectMapper _jsonReader = new ObjectMapper();

  // nt
  private final SendableChooser<Pair<String, Supplier<Command>>> _layoutChooser =
      new SendableChooser<Pair<String, Supplier<Command>>>();
  private final SendableChooser<Pair<String, Supplier<Command>>> _routineChooser =
      new SendableChooser<Pair<String, Supplier<Command>>>();

  private final BooleanSubscriber _useLayoutAuto;

  // generation
  private final AtomicBoolean _shouldGenerate = new AtomicBoolean(false);

  private final AtomicReference<Pair<String, Supplier<Command>>> _latestLayout =
      new AtomicReference<>();
  private final AtomicReference<Pair<String, Supplier<Command>>> _latestRoutine =
      new AtomicReference<>();

  public Auto(
      Shooter shooter,
      Hopper hopper,
      IntakePivot intakePivot,
      IntakeFeed intakeFeed,
      Swerve swerve,
      Superstructure superstructure,
      Supplier<ShotParameters> shotParametersSupplier,
      Consumer<Runnable> addPeriodic) {
    // -- CHOREO --
    _factory =
        new AutoFactory(
            swerve::getPose,
            swerve::resetPose,
            sample -> {
              if (_aimAtTarget) {
                swerve.followTrajectoryFacing(
                    (SwerveSample) sample,
                    Rotation2d.fromRadians(shotParametersSupplier.get().getShotHeading()));
                return;
              }

              swerve.followTrajectory((SwerveSample) sample);
            },
            true,
            swerve,
            (traj, isActive) -> {
              DogLog.log(
                  "Auto/Current Trajectory",
                  FieldUtil.getAlliance() == Alliance.Blue
                      ? traj.getPoses()
                      : traj.flipped().getPoses());
              DogLog.log("Auto/Current Trajectory Name", traj.name());
              DogLog.log("Auto/Current Trajectory Duration", traj.getTotalTime());
              DogLog.log("Auto/Current Trajectory Is Active", isActive);
            });

    _bindings.put(
        "shoot",
        () ->
            sequence(
                runOnce(() -> _aimAtTarget = true), // follow trajectory facing from here
                parallel(hopper.feedShot(), shooter.shoot(), intakePivot.pivotShooting())));

    _bindings.put(
        "stop shooting",
        () ->
            sequence(
                runOnce(() -> _aimAtTarget = false),
                parallel(hopper.feedStop(), shooter.idle(), intakePivot.lower())));

    _bindings.put(
        "shoot still",
        () -> superstructure.shoot(InputStream.zero, InputStream.zero, false).withTimeout(2.5));

    _bindings.put("pivot lower", intakePivot::lower);

    _bindings.put("pivot raise", intakePivot::raise);

    _bindings.put("lower depot enable", () -> runOnce(() -> intakePivot.lowerDepot = true));

    _bindings.put("lower depot disable", () -> runOnce(() -> intakePivot.lowerDepot = false));

    _bindings.put("feed in", intakeFeed::feedIn);

    _bindings.put("feed stop", intakeFeed::feedStop);

    _bindings.forEach((name, command) -> _factory.bind(name, command.get()));

    autonomous().onTrue(runOnce(() -> _aimAtTarget = false));

    // -- NT --
    _useLayoutAuto = DogLog.tunable("Use Layout Auto", true, unused -> _shouldGenerate.set(true));

    logIsAutoGenerated(false);
    logGeneratedAuto("");

    File dir = new File(Filesystem.getDeployDirectory() + "/" + layoutDir);

    for (File layoutFile : dir.listFiles()) {
      try {
        Map<String, Object> layoutJson =
            _jsonReader.readValue(layoutFile, new TypeReference<Map<String, Object>>() {});

        String layoutName = layoutJson.get("name").toString();

        _layoutChooser.addOption(
            layoutName, Pair.of(layoutName, () -> generateLayoutAuto(layoutName, layoutJson)));
      } catch (Exception e) {
        FaultLogger.report(
            "Layout (" + layoutFile.getName() + ") failed to load.", FaultType.ERROR);
      }
    }

    _routineChooser.addOption("Reset Odometry", Pair.of("Reset Odometry", this::resetOdometry));
    _routineChooser.addOption("Preload", Pair.of("Preload", this::preload));
    _routineChooser.addOption("Test", Pair.of("Test", this::test));
    _routineChooser.addOption("Houston", Pair.of("Houston", this::houston));

    _layoutChooser.onChange(
        l -> {
          _latestLayout.set(l);
          _shouldGenerate.set(true);
        });

    _routineChooser.onChange(
        l -> {
          _latestRoutine.set(l);
          _shouldGenerate.set(true);
        });

    SmartDashboard.putData("Layout Chooser", _layoutChooser);
    SmartDashboard.putData("Routine Chooser", _routineChooser);

    // -- GENERATION --
    addPeriodic.accept(
        () -> {
          if (!_shouldGenerate.get()) return;

          logIsAutoGenerated(false);
          logGeneratedAuto("");

          _shouldGenerate.set(false);

          Pair<String, Supplier<Command>> generator =
              _useLayoutAuto.get() ? _latestLayout.get() : _latestRoutine.get();

          if (generator == null) {
            _autoCmd = idle(); // an idle command is used, as a real auto was not generated
            return;
          }

          logGeneratedAuto(generator.getFirst());
          _autoCmd = generator.getSecond().get();

          logIsAutoGenerated(true);
        });

    _latestLayout.set(_layoutChooser.getSelected());
    _latestRoutine.set(_routineChooser.getSelected());

    _shouldGenerate.set(true);
  }

  private void logIsAutoGenerated(boolean isAutoGenerated) {
    DogLog.forceNt.log("Auto/Is Auto Generated", isAutoGenerated);
  }

  private void logGeneratedAuto(String generatedAuto) {
    DogLog.forceNt.log("Auto/Generated Auto", generatedAuto);
  }

  /** Command to run during auto. */
  public Command getAutoScheduler() {
    return deferredProxy(() -> _autoCmd).withName("Auto");
  }

  private Command getBinding(String binding) {
    return _bindings.get(binding).get();
  }

  private Command test() {
    AutoRoutine routine = _factory.newRoutine("Test");
    AutoTrajectory trajectory = routine.trajectory("Test");

    routine.active().onTrue(sequence(trajectory.resetOdometry(), trajectory.cmd()));

    return routine.cmd();
  }

  private Command resetOdometry() {
    AutoRoutine routine = _factory.newRoutine("Reset Odometry");
    AutoTrajectory trajectory = routine.trajectory("ResetOdometry");

    routine.active().onTrue(trajectory.resetOdometry());

    return routine.cmd();
  }

  private Command preload() {
    AutoRoutine routine = _factory.newRoutine("Preload");
    AutoTrajectory trajectory = routine.trajectory("Preload");

    routine.active().onTrue(sequence(trajectory.resetOdometry(), trajectory.cmd()));
    trajectory.done().onTrue(getBinding("shoot still"));

    return routine.cmd();
  }

  private Command houston() {
    AutoRoutine routine = _factory.newRoutine("Houston");
    AutoTrajectory trajectory = routine.trajectory("Houston");

    routine.active().onTrue(sequence(trajectory.resetOdometry(), trajectory.cmd()));
    trajectory.done().onTrue(getBinding("shoot still"));

    return routine.cmd();
  }

  private Command generateLayoutAuto(String layoutName, Map<String, Object> layoutJson) {
    AutoRoutine routine = _factory.newRoutine(layoutName);
    AutoTrajectory trajectory = routine.trajectory(layoutName, 0);

    int splits = routine.trajectory(layoutName).getRawTrajectory().splits().size();

    @SuppressWarnings("unchecked")
    Map<String, List<String>> splitCommands =
        (Map<String, List<String>>) layoutJson.get("splitCommands");

    routine.active().onTrue(sequence(trajectory.resetOdometry(), trajectory.cmd()));

    for (int i = 0; i < splits; i++) {
      SequentialCommandGroup splitCommand = new SequentialCommandGroup();

      if (splitCommands.get(Integer.toString(i)) != null) {
        for (String command : splitCommands.get(Integer.toString(i))) {
          splitCommand.addCommands(getBinding(command));
        }
      }

      if (i + 1 < splits) {
        AutoTrajectory nextTrajectory = routine.trajectory(layoutName, i + 1);
        splitCommand.addCommands(nextTrajectory.spawnCmd());

        trajectory.done().onTrue(splitCommand);
        trajectory = nextTrajectory;

        continue;
      }

      trajectory.done().onTrue(splitCommand);
    }

    return routine.cmd();
  }
}
