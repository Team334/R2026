package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.fasterxml.jackson.databind.ObjectMapper;
import dev.doglog.DogLog;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Swerve;
import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

/** All auton routines. */
public class Autos {
  private final AutoFactory _factory;
  private AutoTrajectory _currentTraj;

  private final Swerve _swerve;

  private SendableChooser<Side> _sideSelector = new SendableChooser<Side>();
  private ObjectMapper _jsonSave = new ObjectMapper();

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

  private final BooleanEntry _shootPreload =
      NetworkTableInstance.getDefault().getBooleanTopic("AutoPreset/Shoot Preload").getEntry(false);
  private final BooleanEntry _depot =
      NetworkTableInstance.getDefault().getBooleanTopic("AutoPreset/Depot").getEntry(false);
  private final BooleanEntry _humanStation =
      NetworkTableInstance.getDefault().getBooleanTopic("AutoPreset/Human Station").getEntry(false);
  private final BooleanEntry _bump =
      NetworkTableInstance.getDefault().getBooleanTopic("AutoPreset/Bump").getEntry(false);
  private final BooleanEntry _neutralZone =
      NetworkTableInstance.getDefault().getBooleanTopic("AutoPreset/Neutral Zone").getEntry(false);
  private final BooleanEntry _climb =
      NetworkTableInstance.getDefault().getBooleanTopic("AutoPreset/Climb").getEntry(false);

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

    _sideSelector.addOption("Left", Side.LEFT);
    _sideSelector.addOption("Center", Side.CENTER);
    _sideSelector.addOption("Right", Side.RIGHT);

    _sideSelector.setDefaultOption("Center", Side.CENTER);

    _shootPreload.set(false);
    _depot.set(false);
    _humanStation.set(false);
    _bump.set(false);
    _neutralZone.set(false);
    _climb.set(false);

    SmartDashboard.putData("Side Selector", _sideSelector);
    SmartDashboard.putData("Save Selected", runOnce(this::saveJson));
    SmartDashboard.putData(
        "Load",
        runOnce(
            () ->
                loadJson(
                    new File(Filesystem.getDeployDirectory() + "/autoPresets/autonLayout.json"))));
  }

  public AutoRoutine example() {
    AutoRoutine routine = _factory.newRoutine("example");

    AutoTrajectory exampleTraj =
        routine.trajectory(_sideSelector.getSelected().getDirectory() + "example");

    routine.active().onTrue(sequence(exampleTraj.resetOdometry(), exampleTraj.cmd()));

    return routine;
  }

  private void saveJson() {
    File folder = new File(Filesystem.getDeployDirectory(), "autoPresets");
    folder.mkdirs();

    File jsonFile = new File(folder, "autonLayout.json");

    Map<String, Boolean> layout = new HashMap<>();

    layout.put("shootPreload", _shootPreload.get());
    layout.put("depot", _depot.get());
    layout.put("humanStation", _humanStation.get());
    layout.put("bump", _bump.get());
    layout.put("neutralZone", _neutralZone.get());
    layout.put("climb", _climb.get());

    try {
      _jsonSave.writerWithDefaultPrettyPrinter().writeValue(jsonFile, layout);
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  private void loadJson(File jsonFile) {
    if (!jsonFile.exists()) {
      System.out.println("File not found: " + jsonFile.getAbsolutePath());
      return;
    }

    Map<String, Boolean> layout;
    try {
      layout =
          _jsonSave.readValue(
              jsonFile,
              new com.fasterxml.jackson.core.type.TypeReference<Map<String, Boolean>>() {});

      _shootPreload.set(layout.getOrDefault("shootPreload", false));
      _depot.set(layout.getOrDefault("depot", false));
      _humanStation.set(layout.getOrDefault("humanStation", false));
      _bump.set(layout.getOrDefault("bump", false));
      _neutralZone.set(layout.getOrDefault("neutralZone", false));
      _climb.set(layout.getOrDefault("climb", false));
    } catch (IOException e) {
      e.printStackTrace();
    }

    System.out.println("Loaded JSON from: " + jsonFile.getAbsolutePath());
  }
}
