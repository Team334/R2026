// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.autonomous;

import com.ctre.phoenix6.SignalLogger;
import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.epilogue.logging.FileBackend;
import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.ClassPreloader;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.FaultLogger;
import frc.lib.FaultsTable.FaultType;
import frc.lib.GcStatsCollector;
import frc.lib.InputStream;
import frc.robot.Constants.Ports;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.Auto;
import frc.robot.commands.Superstructure;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.intake.IntakeFeed;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.utils.FieldUtil;
import frc.robot.utils.ShotParameters;
import java.lang.reflect.Field;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
@Logged(strategy = Strategy.OPT_IN)
public class Robot extends TimedRobot {
  private final NetworkTableInstance _ntInst;

  private boolean _fileOnlySet = false;

  // controllers
  private final CommandXboxController _driverController =
      new CommandXboxController(Ports.driverController);

  @Logged(name = "Shot Parameters")
  private ShotParameters _shotParameters;

  // subsystems
  @Logged(name = "Swerve")
  private final Swerve _swerve = TunerConstants.createDrivetrain();

  @Logged(name = "Shooter")
  private final Shooter _shooter = new Shooter(() -> _shotParameters);

  @Logged(name = "Hopper")
  private final Hopper _hopper = new Hopper(() -> _shotParameters);

  @Logged(name = "IntakePivot")
  private final IntakePivot _intakePivot = new IntakePivot(() -> _shotParameters);

  @Logged(name = "IntakeFeed")
  private final IntakeFeed _intakeFeed = new IntakeFeed();

  @Logged(name = "Climb")
  private final Climb _climb = new Climb();

  private final Superstructure _superstructure =
      new Superstructure(
          _shooter, _hopper, _intakePivot, _intakeFeed, _climb, _swerve, () -> _shotParameters);

  private final Auto _auto =
      new Auto(
          _shooter,
          _hopper,
          _intakePivot,
          _intakeFeed,
          _climb,
          _swerve,
          _superstructure,
          () -> _shotParameters,
          r -> addPeriodic(r, getPeriod()));

  private final Field2d _field2d = new Field2d();

  private final GcStatsCollector _gcStatsCollector = new GcStatsCollector();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    this(NetworkTableInstance.getDefault());
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot(NetworkTableInstance ntInst) {
    super(Constants.robotPeriod.in(Seconds));

    _ntInst = ntInst;

    // set up loggers
    DogLog.setOptions(DogLog.getOptions().withCaptureDs(true));
    DogLog.setPdh(new PowerDistribution());

    setFileOnly(false); // file-only once connected to fms

    Epilogue.bind(this);
    SignalLogger.start();

    DriverStation.silenceJoystickConnectionWarning(isSimulation());

    FaultLogger.setup(_ntInst);

    _shotParameters =
        new ShotParameters(
            () ->
                _shooter.inTolerance()
                    && _shooter
                        .getFlywheelReference()
                        .isNear(_shotParameters.getFlywheelSpeed(), 0.02),
            () ->
                Math.abs(_shotParameters.getShotHeading() - _swerve.getHeading().getRadians())
                    < 0.08,
            () -> FieldUtil.isShotValid(_swerve.getPose()));

    configureDriverBindings();

    new Trigger(() -> _shotParameters.isErrorSensitive)
        .and(_driverController.rightTrigger())
        .whileTrue(
            run(
                () -> {
                  _driverController.setRumble(RumbleType.kBothRumble, 1);
                }))
        .onFalse(
            runOnce(
                () -> {
                  _driverController.setRumble(RumbleType.kBothRumble, 0);
                }));

    // testing tab
    SmartDashboard.putData(
        "Reset Pose", runOnce(() -> _swerve.resetPose(Pose2d.kZero)).ignoringDisable(true));
    SmartDashboard.putData("Reset Heading", _swerve.resetHeading());

    SmartDashboard.putData("Reset Intake Pivot", _intakePivot.reset());

    SmartDashboard.putData("Toggle Field Oriented", _swerve.toggleFieldOriented());

    SmartDashboard.putData("Wheel Radius Characterization", _swerve.wheelRadiusCharacterization());

    SmartDashboard.putData("Calculate Wheel COF", _swerve.calculateWheelCOF());
    SmartDashboard.putData("Calculate Chassis MOI", _swerve.calculateMOI());
    SmartDashboard.putData(
        "Calculate Motor Max Speed And Torque", _swerve.calculateMotorMaxSpeedAndTorque());

    SmartDashboard.putData(
        "Robot Self Check",
        sequence(
                runOnce(() -> DataLogManager.log("Robot Self Check Started")),
                _shooter.fullSelfCheck(),
                _hopper.fullSelfCheck(),
                _intakePivot.fullSelfCheck(),
                _intakeFeed.fullSelfCheck(),
                _climb.fullSelfCheck(),
                _swerve.fullSelfCheck(),
                runOnce(() -> DataLogManager.log("Robot Self Check Finished")))
            .withName("Robot Self Check"));

    SmartDashboard.putData(
        runOnce(FaultLogger::clear).ignoringDisable(true).withName("Clear Faults"));

    // addPeriodic(FaultLogger::update, 1);

    autonomous().whileTrue(_auto.getAutoScheduler());

    preventChoreoDelay();
  }

  /** Watchdog config / class preloading needed to prevent choreo delay. */
  private void preventChoreoDelay() {
    // something slow about watchdog's printEpochs() when there's a loop overrun (Tracer
    // printEpochs() DS writes?)
    // more here: https://www.chiefdelphi.com/t/choreo-autonomous-loop-overruns/495597/21
    final double loopOverrunWarningPeriod = 10;

    try {
      Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
      watchdogField.setAccessible(true);
      Watchdog watchdog = (Watchdog) watchdogField.get(this);
      watchdog.setTimeout(loopOverrunWarningPeriod);
    } catch (Exception e) {
      FaultLogger.report("Failed to increase watchdog timeout", FaultType.ERROR);
    }

    CommandScheduler.getInstance().setPeriod(loopOverrunWarningPeriod);

    // preloading long-loading classes used on auton init by choreo
    ClassPreloader.preload(
        "edu.wpi.first.math.geometry.Transform2d",
        "edu.wpi.first.math.geometry.Twist2d",
        "java.lang.FdLibm$Hypot",
        "choreo.trajectory.Trajectory",
        "choreo.trajectory.SwerveSample");
  }

  // set logging to be file only or not
  private void setFileOnly(boolean fileOnly) {
    DogLog.setOptions(DogLog.getOptions().withNtPublish(!fileOnly));

    if (fileOnly) {
      Epilogue.getConfig().backend = new FileBackend(DataLogManager.getLog());
      return;
    }

    // if doing both file and nt logging, use the datalogger multilogger setup
    Epilogue.getConfig().backend =
        EpilogueBackend.multi(
            new NTEpilogueBackend(_ntInst), new FileBackend(DataLogManager.getLog()));
  }

  private void configureDriverBindings() {
    final InputStream baseVelX =
        InputStream.of(_driverController::getLeftY).deadband(0.02, 1).negate().signedPow(2);

    final InputStream baseVelY =
        InputStream.of(_driverController::getLeftX).deadband(0.02, 1).negate().signedPow(2);

    final InputStream baseVelOmega =
        InputStream.of(_driverController::getRightX).deadband(0.02, 1).negate().signedPow(2);

    _swerve.setDefaultCommand(
        _swerve
            .drive(
                baseVelX.scale(SwerveConstants.driverTranslationalVelocity.in(MetersPerSecond)),
                baseVelY.scale(SwerveConstants.driverTranslationalVelocity.in(MetersPerSecond)),
                baseVelOmega.scale(SwerveConstants.driverAngularVelocity.in(RadiansPerSecond)))
            .beforeStarting(() -> _swerve.isOpenLoop = true));

    final Command shoot =
        _superstructure.shoot(
            baseVelX.scale(SwerveConstants.driverTranslationalShootingVelocity.in(MetersPerSecond)),
            baseVelY.scale(
                SwerveConstants.driverTranslationalShootingVelocity.in(MetersPerSecond)));

    final Command shootManually =
        _superstructure.shootManually(
            baseVelX.scale(SwerveConstants.driverTranslationalShootingVelocity.in(MetersPerSecond)),
            baseVelY.scale(SwerveConstants.driverTranslationalShootingVelocity.in(MetersPerSecond)),
            baseVelOmega.scale(SwerveConstants.driverAngularVelocity.in(RadiansPerSecond)));

    _driverController.rightTrigger().and(() -> !_shotParameters.isManual).whileTrue(shoot);
    _driverController.rightTrigger().and(() -> _shotParameters.isManual).whileTrue(shootManually);

    _driverController.rightBumper().whileTrue(_superstructure.spit());

    _driverController.leftTrigger().whileTrue(_intakeFeed.feedIn());

    _driverController.leftBumper().onTrue(_intakePivot.toggleLowerDefault());

    // _driverController.a().toggleOnTrue(_superstructure.climbRoutine());

    _driverController
        .b()
        .onTrue(runOnce(() -> _shotParameters.isManual = !_shotParameters.isManual));

    _driverController.y().whileTrue(_intakeFeed.feedOut());

    _driverController.x().whileTrue(_superstructure.unjam());
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    DogLog.time("Timing/Robot/robotPeriodic()");

    FieldUtil.getShotParameters(
        _swerve.getPose(),
        ChassisSpeeds.fromRobotRelativeSpeeds(_swerve.getChassisSpeeds(), _swerve.getHeading()),
        _shotParameters);

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    DogLog.time("Timing/Robot/CommandScheduler.getInstance().run()");
    CommandScheduler.getInstance().run();
    DogLog.timeEnd("Timing/Robot/CommandScheduler.getInstance().run()");

    if (DriverStation.isFMSAttached() && !_fileOnlySet) {
      setFileOnly(true);

      _fileOnlySet = true;
    }

    FieldUtil.log(_swerve.getPose());

    DogLog.log(
        "Virtual Target Distance", _shotParameters.getDistanceToVirtualTarget(_swerve.getPose()));

    SmartDashboard.putBoolean("Is Manual", _shotParameters.isManual);
    SmartDashboard.putBoolean("Is Field Oriented", _swerve.isFieldOriented);
    SmartDashboard.putNumber("Shift Time", FieldUtil.getShiftTime());
    SmartDashboard.putNumber("Match Time", FieldUtil.getMatchTime());

    SmartDashboard.putNumber("Shooter Speed", _shooter.getFlywheelSpeed().in(RotationsPerSecond));

    _field2d.setRobotPose(_swerve.getPose());
    SmartDashboard.putData("Field", _field2d);

    _gcStatsCollector.update();

    DogLog.timeEnd("Timing/Robot/robotPeriodic()");

    DogLog.timeEnd("Timing/Robot/Full Loop");
    DogLog.time("Timing/Robot/Full Loop");
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void close() {
    super.close();

    _swerve.close();
    _shooter.close();
    _hopper.close();
    _intakePivot.close();
    _intakeFeed.close();
    _climb.close();
  }
}
