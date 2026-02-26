package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;
import dev.doglog.DogLog;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListenerPoller;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.lib.FaultLogger;
import frc.lib.FaultsTable.FaultType;
import frc.robot.subsystems.Swerve;
import java.io.File;
import java.io.IOException;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

/** Generates the modular auto. */
public class ModularAuto {
  private enum Side {
    LEFT("Left"),
    CENTER("Center"),
    RIGHT("Right");

    private final String _prefix;

    private Side(String prefix) {
      _prefix = prefix;
    }

    public String getPrefix() {
      return _prefix;
    }
  }

  private final AutoFactory _factory;

  private Command _routineCmd;
  private AutoTrajectory _currentTraj;

  private final Swerve _swerve;
  // private final Superstructure _superstructure;

  private final NetworkTableInstance _ntInst;

  // file management
  private final String layoutDir = "layouts";
  private final ObjectMapper _jsonSave = new ObjectMapper();

  // auto table
  private final NetworkTable _autoTable;

  private final BooleanPublisher _isRoutineGenerated;

  private final StringSubscriber _saveLayout;
  private final SendableChooser<String> _chooseLayout = new SendableChooser<String>();

  // layout table
  private final NetworkTable _layoutTable;

  private final SendableChooser<Side> _side;
  private final BooleanEntry _shootPreload;
  private final BooleanEntry _depot;
  private final BooleanEntry _humanStation;
  private final BooleanEntry _bump;
  private final BooleanEntry _neutralZone;
  private final BooleanEntry _climb;

  // nt listeners
  private final NetworkTableListenerPoller _ntPoller;

  private final int _layoutTableListener;
  private final int _saveLayoutListener;

  public ModularAuto(Swerve swerve, Superstructure superstructure, Consumer<Runnable> addPeriodic) {
    _swerve = swerve;
    // _superstructure = superstructure;

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

    _ntInst = NetworkTableInstance.getDefault();

    // build auto table
    _autoTable = _ntInst.getTable("Robot/Auto");

    _isRoutineGenerated = _autoTable.getBooleanTopic("Is Auto Generated").publish();

    _saveLayout = _autoTable.getStringTopic("Save Layout").subscribe("New Layout");
    _autoTable.getStringTopic("Save Layout").publish();

    File dir = new File(Filesystem.getDeployDirectory() + "/" + layoutDir);

    if (dir.listFiles() != null) {
      for (File preset : dir.listFiles()) {
        String fileName = preset.getName().substring(0, preset.getName().lastIndexOf("."));

        _chooseLayout.addOption(fileName, fileName);
      }
    }

    _chooseLayout.setDefaultOption("Empty", "Empty");

    SmartDashboard.putData("Layout Chooser", _chooseLayout);

    // build layout table
    _layoutTable = _autoTable.getSubTable("Layout");

    _side = new SendableChooser<Side>();

    _side.addOption("Left", Side.LEFT);
    _side.addOption("Center", Side.CENTER);
    _side.addOption("Right", Side.RIGHT);

    _side.setDefaultOption("Center", Side.CENTER);

    SmartDashboard.putData("Side Chooser", _side);

    _shootPreload = _layoutTable.getBooleanTopic("Shoot Preload").getEntry(false);
    _depot = _layoutTable.getBooleanTopic("Depot").getEntry(false);
    _humanStation = _layoutTable.getBooleanTopic("Human Station").getEntry(false);
    _bump = _layoutTable.getBooleanTopic("Bump").getEntry(false);
    _neutralZone = _layoutTable.getBooleanTopic("Neutral Zone").getEntry(false);
    _climb = _layoutTable.getBooleanTopic("Climb").getEntry(false);

    _shootPreload.set(false);
    _depot.set(false);
    _humanStation.set(false);
    _bump.set(false);
    _neutralZone.set(false);
    _climb.set(false);

    // set up nt listeners
    _ntPoller = new NetworkTableListenerPoller(_ntInst);

    _layoutTableListener =
        _ntPoller.addListener(
            new String[] {_layoutTable.getPath() + "/"},
            EnumSet.of(NetworkTableEvent.Kind.kValueAll));

    _saveLayoutListener =
        _ntPoller.addListener(_saveLayout, EnumSet.of(NetworkTableEvent.Kind.kValueAll));

    _chooseLayout.onChange(
        layout -> {
          loadLayout(layout);
        });

    _side.onChange(
        side -> {
          generateAuto();
        });

    addPeriodic.accept(this::poll);

    // initial load and generate
    loadLayout(_chooseLayout.getSelected());
    generateAuto();
  }

  private void poll() {
    var changes = _ntPoller.readQueue();

    if (changes.length == 0) return;

    var latestChange = changes[changes.length - 1];

    if (latestChange.listener == _layoutTableListener) {
      generateAuto();
    }

    if (latestChange.listener == _saveLayoutListener) {
      saveLayout(_saveLayout.get());
    }
  }

  /** The auto generated from the specified layout. */
  public Command getAuto() {
    return new ScheduleCommand(_routineCmd).withName("Modular Auto");
  }

  // generate the modular auto
  private void generateAuto() {
    _isRoutineGenerated.set(false);

    AutoRoutine routine = _factory.newRoutine("Modular Auto");

    String obstacle = _bump.get() ? "Bump" : "Trench";

    AutoTrajectory initial = routine.trajectory(_side.getSelected().getPrefix() + "Start");
    AutoTrajectory depot = routine.trajectory(_side.getSelected().getPrefix() + "Depot");
    AutoTrajectory humanStation =
        routine.trajectory(_side.getSelected().getPrefix() + "HumanStation");
    AutoTrajectory neutralZone =
        routine.trajectory(_side.getSelected().getPrefix() + obstacle + "NeutralZone");
    AutoTrajectory climb = routine.trajectory(_side.getSelected().getPrefix() + "Climb");

    _currentTraj = initial;

    routine.active().onTrue(sequence(initial.resetOdometry(), _currentTraj.cmd()));

    if (_shootPreload.get()) {
      // _superstructure.shoot(null, null)
    }

    if (_neutralZone.get()) {
      _currentTraj
          .done()
          .onTrue(sequence(runOnce(() -> _currentTraj = neutralZone), _currentTraj.cmd()));
    }

    if (_depot.get()) {
      _currentTraj
          .done()
          .onTrue(
              sequence(
                  runOnce(() -> _currentTraj = depot),
                  _currentTraj.cmd())); // Add shooting while moving
    }

    if (_humanStation.get()) {
      _currentTraj
          .done()
          .onTrue(
              sequence(
                  runOnce(() -> _currentTraj = humanStation),
                  _currentTraj.cmd())); // Add shooting while moving
    }

    if (_neutralZone.get()) {
      _currentTraj
          .done()
          .onTrue(
              sequence(
                  runOnce(() -> _currentTraj = neutralZone),
                  _currentTraj.cmd())); // Add shooting while moving
    }

    if (_climb.get()) {
      _currentTraj.done().onTrue(sequence(runOnce(() -> _currentTraj = climb), _currentTraj.cmd()));
    }

    _routineCmd = routine.cmd();

    _isRoutineGenerated.set(true);
  }

  // save auto layout as json
  private void saveLayout(String name) {
    File folder = new File(Filesystem.getDeployDirectory(), layoutDir);

    folder.mkdirs();

    File layoutJson = new File(folder, name + ".json");

    Map<String, Boolean> layout = new HashMap<>();

    layout.put("shootPreload", _shootPreload.get());
    layout.put("depot", _depot.get());
    layout.put("humanStation", _humanStation.get());
    layout.put("bump", _bump.get());
    layout.put("neutralZone", _neutralZone.get());
    layout.put("climb", _climb.get());

    try {
      _jsonSave.writerWithDefaultPrettyPrinter().writeValue(layoutJson, layout);

      _chooseLayout.addOption(name, name);

      FaultLogger.report(
          "Saved auto layout (" + layoutJson.getAbsolutePath() + ") successfully.", FaultType.INFO);

    } catch (IOException e) {
      FaultLogger.report(
          "Auto layout (" + layoutJson.getAbsolutePath() + ") failed to save.", FaultType.ERROR);
    }
  }

  // load auto layout from json
  private void loadLayout(String name) {
    File layoutJson =
        new File(Filesystem.getDeployDirectory() + "/" + layoutDir + "/" + name + ".json");

    if (!layoutJson.exists()) {
      FaultLogger.report(
          "Auto layout (" + layoutJson.getAbsolutePath() + ") was not found.", FaultType.ERROR);
      return;
    }

    try {
      Map<String, Boolean> layout =
          _jsonSave.readValue(layoutJson, new TypeReference<Map<String, Boolean>>() {});

      _shootPreload.set(layout.getOrDefault("shootPreload", false));
      _depot.set(layout.getOrDefault("depot", false));
      _humanStation.set(layout.getOrDefault("humanStation", false));
      _bump.set(layout.getOrDefault("bump", false));
      _neutralZone.set(layout.getOrDefault("neutralZone", false));
      _climb.set(layout.getOrDefault("climb", false));

      FaultLogger.report(
          "Loaded auto layout (" + layoutJson.getAbsolutePath() + ") successfully.",
          FaultType.INFO);
    } catch (IOException e) {
      FaultLogger.report(
          "Auto layout (" + layoutJson.getAbsolutePath() + ") failed to load.", FaultType.ERROR);
    }
  }
}
