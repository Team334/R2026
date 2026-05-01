// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest.*;
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;
import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.InputStream;
import frc.lib.SysId;
import frc.lib.fault.FaultLogger;
import frc.lib.fault.FaultsTable;
import frc.lib.fault.FaultsTable.Fault;
import frc.lib.fault.FaultsTable.FaultType;
import frc.lib.math.TimeoutLinearFilter;
import frc.lib.subsystem.SelfChecked;
import frc.lib.util.CTREUtil;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.utils.HolonomicController;
import frc.robot.utils.VisionPoseEstimator;
import frc.robot.utils.VisionPoseEstimator.VisionPoseEstimate;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import org.photonvision.simulation.VisionSystemSim;

public class Swerve extends TunerSwerveDrivetrain implements Subsystem, SelfChecked {
  // teleop requests
  private final RobotCentric _robotCentricRequest = new RobotCentric();
  private final FieldCentric _fieldCentricRequest = new FieldCentric();

  private final SwerveDriveBrake _brakeRequest = new SwerveDriveBrake();

  // auton request for choreo / pose controller
  private final ApplyFieldSpeeds _fieldSpeedsRequest = new ApplyFieldSpeeds();

  @Logged(name = "Holonomic Controller")
  private final HolonomicController _holonomicController =
      new HolonomicController(getKinematics().getModules());

  // for drive facing
  private final TimeoutLinearFilter _omegaFeedforwardFilter =
      new TimeoutLinearFilter(
          LinearFilter.singlePoleIIR(0.1, Constants.robotPeriod.in(Seconds)), 2);
  private Rotation2d _previousRotationSetpoint = Rotation2d.kZero;

  private double _lastSimTime = 0;
  private Notifier _simNotifier;

  // faults and the table containing them
  private Set<Fault> _faults = new HashSet<Fault>();
  private FaultsTable _faultsTable =
      new FaultsTable(
          NetworkTableInstance.getDefault().getTable("SelfChecked"),
          getName() + " Faults"); // TODO: watch out unit tests

  private boolean _hasError = false;

  private final SysIdSwerveTranslation _translationCharacterization = new SysIdSwerveTranslation();
  private final SysIdSwerveSteerGains _steerCharacterization = new SysIdSwerveSteerGains();
  private final SysIdSwerveRotation _rotationCharacterization = new SysIdSwerveRotation();

  private final SysIdRoutine _translationRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> setControl(_translationCharacterization.withVolts(output)), null, this));

  private final SysIdRoutine _steerRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(7), // Use dynamic voltage of 7 V
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(_steerCharacterization.withVolts(volts)), null, this));

  private final SysIdRoutine _rotationRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              /* This is in radians per second², but SysId only supports "volts per second" */
              Volts.of(Math.PI / 6).per(Second),
              /* This is in radians per second, but SysId only supports "volts" */
              Volts.of(Math.PI),
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> {
                // output is actually radians per second, but SysId only supports "volts"
                setControl(_rotationCharacterization.withRotationalRate(output.in(Volts)));
                // also log the requested output for SysId
                SignalLogger.writeDouble("rotational_rate", output.in(Volts));
              },
              null,
              this));

  @Logged(name = "Is Field Oriented")
  public boolean isFieldOriented = true;

  @Logged(name = "Is Open Loop")
  public boolean isOpenLoop = true;

  @Logged(name = "Ignore Vision Estimates")
  private boolean _ignoreVisionEstimates = false;

  @Logged(name = VisionConstants.leftArducamName)
  private final VisionPoseEstimator _leftArducam =
      VisionPoseEstimator.buildFromConstants(VisionConstants.leftArducam, this::getHeadingAtTime);

  @Logged(name = VisionConstants.rightArducamName)
  private final VisionPoseEstimator _rightArducam =
      VisionPoseEstimator.buildFromConstants(VisionConstants.rightArducam, this::getHeadingAtTime);

  private final List<VisionPoseEstimator> _cameras = List.of(_leftArducam, _rightArducam);

  private final List<VisionPoseEstimate> _newEstimates = new ArrayList<>();

  private final List<VisionPoseEstimate> _acceptedEstimates = new ArrayList<>();
  private final List<VisionPoseEstimate> _rejectedEstimates = new ArrayList<>();

  private Matrix<N3, N1> _visionStdDevs = new Matrix<>(Nat.N3(), Nat.N1());

  private final Set<Pose3d> _detectedTags = new HashSet<>();

  private final VisionSystemSim _visionSystemSim;

  private boolean _hasAppliedDriverPerspective = false;

  /**
   * Creates a new Swerve.
   *
   * @param drivetrainConstants The CTRE {@link SwerveDrivetrainConstants}. These involve the CAN
   *     Bus name and the Pigeon Id.
   * @param moduleConstants The CTRE {@link SwerveModuleConstants}. The involve constants identical
   *     across all modules (PID constants, wheel radius, etc), and constants unique to each module
   *     (location, device ids, etc).
   */
  public Swerve(
      SwerveDrivetrainConstants drivetrainConstants,
      SwerveModuleConstants<?, ?, ?>... moduleConstants) {
    super(drivetrainConstants, SwerveConstants.odometryFrequency.in(Hertz), moduleConstants);

    // final var frontLeftDriveConfigurator = getModule(0).getDriveMotor().getConfigurator();
    // final var frontRightDriveConfigurator = getModule(1).getDriveMotor().getConfigurator();

    // final var currentLimitConfigs = new CurrentLimitsConfigs();

    // CTREUtil.attempt(
    //     () -> frontLeftDriveConfigurator.refresh(currentLimitConfigs),
    //     getModule(0).getDriveMotor());
    // currentLimitConfigs.StatorCurrentLimit = SwerveConstants.frontSlipCurrent.in(Amps);
    // CTREUtil.attempt(
    //     () -> frontLeftDriveConfigurator.apply(currentLimitConfigs),
    // getModule(0).getDriveMotor());

    // CTREUtil.attempt(
    //     () -> frontRightDriveConfigurator.refresh(currentLimitConfigs),
    //     getModule(1).getDriveMotor());
    // currentLimitConfigs.StatorCurrentLimit = SwerveConstants.frontSlipCurrent.in(Amps);
    // CTREUtil.attempt(
    //     () -> frontRightDriveConfigurator.apply(currentLimitConfigs),
    // getModule(1).getDriveMotor());

    _robotCentricRequest
        .withDeadband(SwerveConstants.translationalDeadband)
        .withRotationalDeadband(SwerveConstants.rotationalDeadband);

    _fieldCentricRequest
        .withDeadband(SwerveConstants.translationalDeadband)
        .withRotationalDeadband(SwerveConstants.rotationalDeadband);

    // closed loop vel always in auto
    _fieldSpeedsRequest.withDriveRequestType(DriveRequestType.Velocity);

    SysId.displayRoutine("Swerve Translation", _translationRoutine);
    SysId.displayRoutine("Swerve Steer", _steerRoutine);
    SysId.displayRoutine("Swerve Rotation", _rotationRoutine);

    registerFallibles();

    if (Robot.isSimulation()) {
      startSimThread();

      _visionSystemSim = new VisionSystemSim("main");
      _visionSystemSim.addAprilTags(FieldConstants.tagLayout);

      _cameras.forEach(cam -> _visionSystemSim.addCamera(cam.getCameraSim(), cam.robotToCam));
    } else {
      _visionSystemSim = null;
    }
  }

  // COPIED FROM ADVANCED SUBSYSTEM

  /**
   * Returns the name of the command that's currently requiring this subsystem. Is "None" when the
   * command in null.
   */
  @Logged(name = "Current Command")
  public final String currentCommandName() {
    if (getCurrentCommand() != null) {
      return getCurrentCommand().getName();
    }

    return "None";
  }

  /** Adds a new fault under this subsystem. */
  public final void addFault(String description, FaultType faultType) {
    _hasError = (faultType == FaultType.ERROR);

    Fault fault = new Fault(description, faultType);

    DogLog.logFault(fault.toString(), null);

    _faults.add(fault);
    _faultsTable.set(_faults);
  }

  /** Clears this subsystem's faults. */
  public final void clearFaults() {
    _faults.clear();
    _faultsTable.set(_faults);

    _hasError = false;
  }

  /** Returns the faults belonging to this subsystem. */
  public final Set<Fault> getFaults() {
    return _faults;
  }

  /** Returns whether this subsystem contains the following fault. */
  public final boolean hasFault(String description, FaultType faultType) {
    return _faults.contains(new Fault(description, faultType));
  }

  /** Returns whether this subsystem has errors (has fault type of error). */
  @Logged(name = "Has Error")
  public final boolean hasError() {
    return _hasError;
  }

  /** Returns a full Command that self checks this Subsystem for pre-match. */
  public final Command fullSelfCheck() {
    Command selfCheck =
        sequence(runOnce(this::clearFaults), selfCheck().until(this::hasError))
            .withName(getName() + " Self Check");
    return selfCheck;
  }

  private void registerFallibles() {
    for (SwerveModule<TalonFX, TalonFX, CANcoder> module : getModules()) {
      FaultLogger.register(module.getDriveMotor());
      FaultLogger.register(module.getSteerMotor());
      FaultLogger.register(module.getEncoder());
    }

    FaultLogger.register(getPigeon2());
  }

  private void startSimThread() {
    _lastSimTime = Utils.getCurrentTimeSeconds();

    // Run simulation at a faster rate so PID gains behave more reasonably
    _simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - _lastSimTime;
              _lastSimTime = currentTime;

              // use the measured time delta, get battery voltage from WPILib
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });

    _simNotifier.setName("Swerve Sim Thread");
    _simNotifier.startPeriodic(1 / Constants.simNotifierFrequency.in(Hertz));
  }

  /** Toggles the field oriented boolean. */
  public Command toggleFieldOriented() {
    return brake()
        .withTimeout(0.5)
        .andThen(runOnce(() -> isFieldOriented = !isFieldOriented))
        .withName("Toggle Field Oriented");
  }

  /** Brakes the swerve drive (modules form an "X" formation). */
  public Command brake() {
    return run(() -> setControl(_brakeRequest)).withName("Brake");
  }

  /** Resets the heading to face away from the alliance wall. */
  public Command resetHeading() {
    return runOnce(
            () -> {
              Rotation2d rotation =
                  DriverStation.getAlliance()
                      .map(
                          allianceColor ->
                              allianceColor == Alliance.Red ? Rotation2d.k180deg : Rotation2d.kZero)
                      .orElse(Rotation2d.kZero);

              resetPose(new Pose2d(getPose().getTranslation(), rotation));
            })
        .ignoringDisable(true)
        .withName("Reset Heading");
  }

  /**
   * Drives the chassis in teleop, with the chassis fixed at a supplied heading. Open loop / field
   * oriented behavior is configured with {@link #isOpenLoop} and {@link #isFieldOriented}.
   *
   * @param velX The x velocity in meters per second.
   * @param velY The y velocity in meters per second.
   * @param heading The heading the chassis should drive at.
   */
  public Command driveFacing(InputStream velX, InputStream velY, Supplier<Rotation2d> heading) {
    return drive(
            velX,
            velY,
            () -> {
              double omegaFeedforward =
                  MathUtil.inputModulus(
                          heading.get().getRadians() - _previousRotationSetpoint.getRadians(),
                          -Math.PI,
                          Math.PI)
                      / Constants.robotPeriod.in(Seconds);

              // filter out spikes in heading.get() derivative
              // (auton hack for now for snm)
              if (DriverStation.isTeleop()) {
                omegaFeedforward = _omegaFeedforwardFilter.calculate(omegaFeedforward);
              }

              _previousRotationSetpoint = heading.get();

              double omega =
                  _holonomicController.calculate(
                              new Pose2d(getPose().getTranslation(), heading.get()), getPose())
                          .omegaRadiansPerSecond
                      + omegaFeedforward;

              return omega;
            })
        .beforeStarting(
            runOnce(
                () -> {
                  isOpenLoop = false;

                  // auton hack for now for snm
                  _holonomicController.useFilteringHeading(DriverStation.isTeleop());
                }))
        .withName("Drive Facing");
  }

  /**
   * Drives the chassis in teleop. Open loop / field oriented behavior is configured with {@link
   * #isOpenLoop} and {@link #isFieldOriented}.
   *
   * @param velX The x velocity in meters per second.
   * @param velY The y velocity in meters per second.
   * @param velOmega The rotational velocity in radians per second.
   */
  public Command drive(InputStream velX, InputStream velY, InputStream velOmega) {
    return run(() -> {
          if (isFieldOriented) {
            setControl(
                _fieldCentricRequest
                    .withVelocityX(velX.get())
                    .withVelocityY(velY.get())
                    .withRotationalRate(velOmega.get())
                    .withDriveRequestType(
                        isOpenLoop ? DriveRequestType.OpenLoopVoltage : DriveRequestType.Velocity));
          } else {
            setControl(
                _robotCentricRequest
                    .withVelocityX(velX.get())
                    .withVelocityY(velY.get())
                    .withRotationalRate(velOmega.get())
                    .withDriveRequestType(
                        isOpenLoop ? DriveRequestType.OpenLoopVoltage : DriveRequestType.Velocity));
          }
        })
        .withName("Drive");
  }

  /**
   * Sets the chassis state to the given {@link SwerveSample} for trajectory following.
   *
   * @param sample The SwerveSample.
   */
  public void followTrajectory(SwerveSample sample) {
    _holonomicController.useFilteringTranslation(false);
    _holonomicController.useFilteringHeading(false);

    ChassisSpeeds desiredSpeeds = sample.getChassisSpeeds();
    Pose2d desiredPose = sample.getPose();

    desiredSpeeds = _holonomicController.calculate(desiredSpeeds, desiredPose, getPose());

    setControl(
        _fieldSpeedsRequest
            .withSpeeds(desiredSpeeds)
            .withWheelForceFeedforwardsX(sample.moduleForcesX())
            .withWheelForceFeedforwardsY(sample.moduleForcesY()));
  }

  /**
   * Sets the chassis state to the given {@link SwerveSample} for trajectory following. Omega is
   * overriden by the omega generated from the holonomic controller to follow the supplied heading.
   *
   * @param sample The SwerveSample.
   * @param heading Heading to follow.
   */
  public void followTrajectoryFacing(SwerveSample sample, Rotation2d heading) {
    _holonomicController.useFilteringTranslation(false);
    _holonomicController.useFilteringHeading(true);

    ChassisSpeeds desiredSpeeds = sample.getChassisSpeeds();

    desiredSpeeds.omegaRadiansPerSecond =
        MathUtil.inputModulus(
                heading.getRadians() - _previousRotationSetpoint.getRadians(), -Math.PI, Math.PI)
            / Constants.robotPeriod.in(Seconds);

    // filter out spikes in heading derivative
    desiredSpeeds.omegaRadiansPerSecond =
        _omegaFeedforwardFilter.calculate(desiredSpeeds.omegaRadiansPerSecond);
    _previousRotationSetpoint = heading;

    desiredSpeeds =
        _holonomicController.calculate(
            desiredSpeeds, new Pose2d(sample.x, sample.y, heading), getPose());

    // assume shot heading second deriv is low (and choreo accel is low) and set alpha to 0
    Feedforwards sampleLinearForces =
        _holonomicController.getWheelForceCalculator().calculate(sample.ax, sample.ay, 0);

    setControl(
        _fieldSpeedsRequest
            .withSpeeds(desiredSpeeds)
            .withWheelForceFeedforwardsX(sampleLinearForces.x_newtons)
            .withWheelForceFeedforwardsY(sampleLinearForces.y_newtons));
  }

  /** Drives the robot in a straight line to some given goal pose. */
  public Command driveTo(Pose2d goalPose) {
    return driveTo(() -> goalPose);
  }

  /** Drives the robot in a straight line to some given goal pose. */
  public Command driveTo(Supplier<Pose2d> goalPose) {
    return run(() -> {
          _holonomicController.nextSetpoint(getHeading());

          ChassisSpeeds desiredSpeeds =
              _holonomicController.calculate(
                  _holonomicController.getSetpointSpeeds(),
                  _holonomicController.getSetpointPose(),
                  getPose());

          setControl(
              _fieldSpeedsRequest
                  .withSpeeds(desiredSpeeds)
                  .withWheelForceFeedforwardsX(_holonomicController.getWheelForces().x_newtons)
                  .withWheelForceFeedforwardsY(_holonomicController.getWheelForces().y_newtons));
        })
        .beforeStarting(
            () -> {
              _holonomicController.useFilteringTranslation(false);
              _holonomicController.useFilteringHeading(false);

              _holonomicController.reset(
                  getPose(),
                  goalPose.get(),
                  ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getHeading()));
            })
        .until(_holonomicController::isFinished)
        .withName("Drive To");
  }

  /** Wrapper for getting estimated pose. */
  public Pose2d getPose() {
    return getState().Pose;
  }

  /** Wrapper for getting estimated heading. */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /** Returns the robot's estimated rotation at the given timestamp (FPGA time). */
  public Rotation2d getHeadingAtTime(double timestamp) {
    return samplePoseAt(Utils.fpgaToCurrentTime(timestamp)).orElse(getPose()).getRotation();
  }

  /** Wrapper for getting current robot-relative chassis speeds. */
  public ChassisSpeeds getChassisSpeeds() {
    return getState().Speeds;
  }

  // updates pose estimator with vision
  private void updateVisionPoseEstimates() {
    _newEstimates.clear();

    _acceptedEstimates.clear();
    _rejectedEstimates.clear();

    _detectedTags.clear();

    for (VisionPoseEstimator cam : _cameras) {
      cam.update();

      var estimates = cam.getNewEstimates();

      // add estimates to arrays and update detected tags
      estimates.forEach(
          (estimate) -> {
            // add all detected tag poses
            for (int id : estimate.detectedTags()) {
              FieldConstants.tagLayout.getTagPose(id).ifPresent(pose -> _detectedTags.add(pose));
            }

            // add robot poses to their corresponding arrays
            if (estimate.isValid()) _acceptedEstimates.add(estimate);
            else _rejectedEstimates.add(estimate);
          });

      _newEstimates.addAll(_acceptedEstimates);
      _newEstimates.addAll(_rejectedEstimates);
    }
  }

  @Override
  public void periodic() {
    DogLog.time("Timing/Swerve/periodic()");

    updateVisionPoseEstimates();

    if (!_hasAppliedDriverPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red ? Rotation2d.k180deg : Rotation2d.kZero);

                _hasAppliedDriverPerspective = true;
              });
    }

    if (!_ignoreVisionEstimates) {
      _acceptedEstimates.sort(VisionPoseEstimate.sorter);

      _acceptedEstimates.forEach(
          (e) -> {
            var stdDevs = e.stdDevs();
            _visionStdDevs.set(0, 0, stdDevs[0]);
            _visionStdDevs.set(1, 0, stdDevs[1]);
            _visionStdDevs.set(2, 0, stdDevs[2]);

            addVisionMeasurement(
                e.pose().toPose2d(), Utils.fpgaToCurrentTime(e.timestamp()), _visionStdDevs);
          });
    }

    SwerveDriveState state = getState();

    DogLog.log("Swerve/Pose", state.Pose);
    DogLog.log("Swerve/Raw Heading", state.RawHeading);
    DogLog.log("Swerve/Speeds", state.Speeds);
    DogLog.log("Swerve/Desired Speeds", getKinematics().toChassisSpeeds(state.ModuleTargets));
    DogLog.log("Swerve/Module States", state.ModuleStates);
    DogLog.log("Swerve/Desired Module States", state.ModuleTargets);

    double totalDaqs = state.SuccessfulDaqs + state.FailedDaqs;
    totalDaqs = totalDaqs == 0 ? 1 : totalDaqs;

    DogLog.log("Swerve/Odometry Success %", state.SuccessfulDaqs / totalDaqs * 100);
    DogLog.log("Swerve/Odometry Period", state.OdometryPeriod);

    DogLog.log(
        "Swerve/Accepted Estimates",
        _acceptedEstimates.stream().map(VisionPoseEstimate::pose).toArray(Pose3d[]::new));
    DogLog.log(
        "Swerve/Rejected Estimates",
        _rejectedEstimates.stream().map(VisionPoseEstimate::pose).toArray(Pose3d[]::new));

    DogLog.log("Swerve/Detected Tags", _detectedTags.toArray(Pose3d[]::new));

    DogLog.log(getName() + "/Current Command", currentCommandName());
    DogLog.log(getName() + "/Has Error", _hasError);

    DogLog.timeEnd("Timing/Swerve/periodic()");
  }

  @Override
  public void simulationPeriodic() {
    _visionSystemSim.update(getPose());
  }

  /** Calculates the chassis MOI given angular chassis kA in volts/rad/s^2 per each module. */
  public Command calculateMOI() {
    // volts/rad/s^2 for each individual drive motor in the correct order
    final DoubleArraySubscriber angularkAPerModule =
        DogLog.tunable("Calculator/Angular kA Per Module", new double[4]);

    final DCMotor driveMotor = MotorConstants.krakenX60;

    return Commands.runOnce(
            () -> {
              double chassisTorque = 0;

              for (int module = 0; module < 4; module++) {
                double moduleToCenter = getModuleLocations()[module].getNorm();
                double moduleForce =
                    driveMotor.getTorque(driveMotor.getCurrent(0, angularkAPerModule.get()[module]))
                        * TunerConstants.FrontLeft.DriveMotorGearRatio
                        / TunerConstants.FrontLeft.WheelRadius;

                chassisTorque += moduleForce * moduleToCenter;
              }

              double MOI = chassisTorque; // since α = 1, I = τ

              FaultLogger.report("Chassis MOI: " + MOI, FaultType.INFO);
            })
        .ignoringDisable(true)
        .withName("Calculate Chassis MOI");
  }

  /**
   * Calculate the drive motor max speed and torque accounting for pose PID headroom and stator
   * current limit.
   */
  public Command calculateMotorMaxSpeedAndTorque() {
    // percentage of max achievable speed and torque, leaving headroom for pose PID
    final DoubleSubscriber headroom = DogLog.tunable("Calculator/Headroom", .8);

    final DCMotor driveMotor = MotorConstants.krakenX60;

    final CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();

    return Commands.runOnce(
            () -> {
              double maxSpeed =
                  Units.radiansPerSecondToRotationsPerMinute(
                      driveMotor.freeSpeedRadPerSec * headroom.get());

              double maxTorque = 0;

              for (var module : getModules()) {
                var failed =
                    CTREUtil.attempt(
                        () ->
                            module.getDriveMotor().getConfigurator().refresh(currentLimitsConfigs),
                        module.getDriveMotor());

                if (failed) {
                  FaultLogger.report(
                      "Failed to calculate motor max torque! Make sure all drive motors are connected.",
                      FaultType.ERROR);
                  return;
                }

                maxTorque +=
                    driveMotor.getTorque(currentLimitsConfigs.StatorCurrentLimit) * headroom.get();
              }

              maxTorque /= 4;

              FaultLogger.report(
                  "Motor Max Speed (rpm): " + maxSpeed + ", Motor Max Torque: " + maxTorque,
                  FaultType.INFO);
            })
        .ignoringDisable(true)
        .withName("Calculate Motor Max Speed And Torque");
  }

  // returns the distance traveled by each individual drive wheel in radians
  private double[] getWheelDistancesRadians() {
    SwerveModulePosition[] positions = getState().ModulePositions;

    double[] distances = new double[4];

    for (int i = 0; i < getModules().length; i++) {
      // radians = (meters / radius)
      distances[i] = positions[i].distanceMeters / TunerConstants.FrontLeft.WheelRadius;
    }

    return distances;
  }

  private class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public Command wheelRadiusCharacterization() {
    final Distance driveRadius =
        Meters.of(
            Math.sqrt(
                Math.pow(TunerConstants.FrontLeft.LocationX, 2)
                    + Math.pow(TunerConstants.FrontLeft.LocationY, 2)));

    final AngularAcceleration angularAcceleration = RadiansPerSecondPerSecond.of(0.05);
    final AngularVelocity angularVelocity = RadiansPerSecond.of(0.25);

    SlewRateLimiter limiter =
        new SlewRateLimiter(angularAcceleration.in(RadiansPerSecondPerSecond));
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
            // Drive control sequence
            Commands.sequence(
                // Reset acceleration limiter
                Commands.runOnce(
                    () -> {
                      limiter.reset(0.0);
                    }),

                // Turn in place, accelerating up to full speed
                run(
                    () -> {
                      double speed = limiter.calculate(angularVelocity.in(RadiansPerSecond));
                      setControl(_fieldSpeedsRequest.withSpeeds(new ChassisSpeeds(0, 0, speed)));
                    })),

            // Measurement sequence
            Commands.sequence(
                // Wait for modules to fully orient before starting measurement
                Commands.waitSeconds(1.5),

                // Record starting measurement
                Commands.runOnce(
                    () -> {
                      state.positions = getWheelDistancesRadians();
                      state.lastAngle = getHeading();
                      state.gyroDelta = 0.0;
                    }),

                // Update gyro delta
                Commands.run(
                        () -> {
                          var rotation = getHeading();
                          state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                          state.lastAngle = rotation;
                        })

                    // When cancelled, calculate and print results
                    .finallyDo(
                        () -> {
                          double[] positions = getWheelDistancesRadians();
                          double wheelDelta = 0.0;
                          for (int i = 0; i < 4; i++) {
                            wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                          }
                          double wheelRadius =
                              (state.gyroDelta * driveRadius.in(Meters)) / wheelDelta;

                          FaultLogger.report(
                              "Wheel Radius Inches: " + Units.metersToInches(wheelRadius),
                              FaultType.INFO);
                        })))
        .withName("Wheel Radius Characterization");
  }

  private Command selfCheckModule(String name, SwerveModule<TalonFX, TalonFX, CANcoder> module) {
    return shiftSequence();
  }

  @Override
  public Command selfCheck() {
    return shiftSequence(
        // check all modules individually
        selfCheckModule("Front Left", getModule(0)),
        selfCheckModule("Front Right", getModule(1)),
        selfCheckModule("Back Left", getModule(2)),
        selfCheckModule("Back Right", getModule(3)));
  }

  @Override
  public void close() {
    super.close();

    _cameras.forEach(c -> c.close());

    _simNotifier.close();
  }
}
