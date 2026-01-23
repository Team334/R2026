// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.*;
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;
import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
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
import frc.lib.FaultLogger;
import frc.lib.FaultsTable;
import frc.lib.FaultsTable.Fault;
import frc.lib.FaultsTable.FaultType;
import frc.lib.InputStream;
import frc.lib.SelfChecked;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.utils.HolonomicController;
import frc.robot.utils.SysId;
import frc.robot.utils.VisionPoseEstimator;
import frc.robot.utils.VisionPoseEstimator.VisionPoseEstimate;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import org.photonvision.simulation.VisionSystemSim;

@Logged(strategy = Strategy.OPT_IN)
public class Swerve extends TunerSwerveDrivetrain implements Subsystem, SelfChecked {
  // teleop requests
  private final RobotCentric _robotCentricRequest = new RobotCentric();
  private final FieldCentric _fieldCentricRequest = new FieldCentric();

  private final SwerveDriveBrake _brakeRequest = new SwerveDriveBrake();

  // auton request for choreo / pose controller
  private final ApplyFieldSpeeds _fieldSpeedsRequest = new ApplyFieldSpeeds();

  private final HolonomicController _poseController =
      new HolonomicController(getKinematics().getModules());

  private double _lastSimTime = 0;
  private Notifier _simNotifier;

  // faults and the table containing them
  private Set<Fault> _faults = new HashSet<Fault>();
  private FaultsTable _faultsTable =
      new FaultsTable(
          NetworkTableInstance.getDefault().getTable("Self Check"),
          getName() + " Faults"); // TODO: watch out unit tests

  private boolean _hasError = false;

  private final SwerveRequest.SysIdSwerveTranslation _translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains _steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation _rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();

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

  @Logged(name = "Driver Chassis Speeds")
  private final ChassisSpeeds _driverChassisSpeeds = new ChassisSpeeds();

  @Logged(name = "Ignore Vision Estimates")
  private boolean _ignoreVisionEstimates = false;

  private final List<VisionPoseEstimator> _cameras = List.of();

  private final List<VisionPoseEstimate> _newEstimates = new ArrayList<>();

  private final List<VisionPoseEstimate> _acceptedEstimates = new ArrayList<>();
  private final List<VisionPoseEstimate> _rejectedEstimates = new ArrayList<>();

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

    _robotCentricRequest
        .withDeadband(SwerveConstants.translationalDeadband)
        .withRotationalDeadband(SwerveConstants.rotationalDeadband);

    _fieldCentricRequest
        .withDeadband(SwerveConstants.translationalDeadband)
        .withRotationalDeadband(SwerveConstants.rotationalDeadband);

    // closed loop vel always in auto
    _fieldSpeedsRequest.withDriveRequestType(DriveRequestType.Velocity);

    registerTelemetry(
        state -> {
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
        });

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
  private final void addFault(String description, FaultType faultType) {
    _hasError = (faultType == FaultType.ERROR);

    Fault fault = new Fault(description, faultType);

    DogLog.logFault(fault.toString());

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
        });
  }

  /**
   * Creates a new command that drives the chassis in teleop.
   *
   * @param velX The x velocity in meters per second.
   * @param velY The y velocity in meters per second.
   * @param velOmega The rotational velocity in radians per second.
   */
  public Command drive(InputStream velX, InputStream velY, InputStream velOmega) {
    return run(() -> {
          drive(velX.get(), velY.get(), velOmega.get());
        })
        .withName("Drive");
  }

  /**
   * Drives the chassis in teleop. Open loop / field oriented behavior is configured with {@link
   * #isOpenLoop} and {@link #isFieldOriented}.
   *
   * @param velX The x velocity in meters per second.
   * @param velY The y velocity in meters per second.
   * @param velOmega The rotational velocity in radians per second.
   */
  public void drive(double velX, double velY, double velOmega) {
    _driverChassisSpeeds.vxMetersPerSecond = velX;
    _driverChassisSpeeds.vyMetersPerSecond = velY;
    _driverChassisSpeeds.omegaRadiansPerSecond = velOmega;

    if (isFieldOriented) {
      setControl(
          _fieldCentricRequest
              .withVelocityX(velX)
              .withVelocityY(velY)
              .withRotationalRate(velOmega)
              .withDriveRequestType(
                  isOpenLoop ? DriveRequestType.OpenLoopVoltage : DriveRequestType.Velocity));
    } else {
      setControl(
          _robotCentricRequest
              .withVelocityX(velX)
              .withVelocityY(velY)
              .withRotationalRate(velOmega)
              .withDriveRequestType(
                  isOpenLoop ? DriveRequestType.OpenLoopVoltage : DriveRequestType.Velocity));
    }
  }

  /**
   * Sets the chassis state to the given {@link SwerveSample} for trajectory following.
   *
   * @param sample The SwerveSample.
   */
  public void followTrajectory(SwerveSample sample) {
    var desiredSpeeds = sample.getChassisSpeeds();
    var desiredPose = sample.getPose();

    desiredSpeeds = _poseController.calculate(desiredSpeeds, desiredPose, getPose());

    setControl(
        _fieldSpeedsRequest
            .withSpeeds(desiredSpeeds)
            .withWheelForceFeedforwardsX(sample.moduleForcesX())
            .withWheelForceFeedforwardsY(sample.moduleForcesY()));
  }

  /** Drives the robot in a straight line to some given goal pose. */
  public Command driveTo(Pose2d goalPose) {
    return driveTo(() -> goalPose);
  }

  /** Drives the robot in a straight line to some given goal pose. */
  public Command driveTo(Supplier<Pose2d> goalPose) {
    return run(() -> {
          ChassisSpeeds speeds = _poseController.calculate(getPose());
          Feedforwards wheelForces = _poseController.getWheelForces();

          setControl(
              _fieldSpeedsRequest
                  .withSpeeds(speeds)
                  .withWheelForceFeedforwardsX(wheelForces.x_newtons)
                  .withWheelForceFeedforwardsY(wheelForces.y_newtons));
        })
        .beforeStarting(
            () ->
                _poseController.reset(
                    getPose(),
                    goalPose.get(),
                    ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getHeading())))
        .until(_poseController::isFinished)
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

    DogLog.log(
        "Swerve/Accepted Estimates",
        _acceptedEstimates.stream().map(VisionPoseEstimate::pose).toArray(Pose3d[]::new));
    DogLog.log(
        "Swerve/Rejected Estimates",
        _rejectedEstimates.stream().map(VisionPoseEstimate::pose).toArray(Pose3d[]::new));

    DogLog.log("Swerve/Detected Tags", _detectedTags.toArray(Pose3d[]::new));

    if (!_ignoreVisionEstimates) {
      _acceptedEstimates.sort(VisionPoseEstimate.sorter);

      _acceptedEstimates.forEach(
          (e) -> {
            var stdDevs = e.stdDevs();
            addVisionMeasurement(
                e.pose().toPose2d(),
                Utils.fpgaToCurrentTime(e.timestamp()),
                VecBuilder.fill(stdDevs[0], stdDevs[1], stdDevs[2]));
          });
    }

    DogLog.timeEnd("Timing/Swerve/periodic()");
  }

  @Override
  public void simulationPeriodic() {
    _visionSystemSim.update(getPose()); // TODO: odom only?
  }

  /** Calculates the chassis MOI given angular chassis kA in volts/rad/s^2. */
  public Command calculateMOI() {
    final Distance driveRadius =
        Meters.of(
            Math.sqrt(
                Math.pow(TunerConstants.FrontLeft.LocationX, 2)
                    + Math.pow(TunerConstants.FrontLeft.LocationY, 2)));

    final DoubleSubscriber angularkA = DogLog.tunable("Angular kA", 1.0);

    return Commands.runOnce(
            () -> {
              DCMotor driveMotor = DCMotor.getKrakenX60(1);

              double chassisTorque =
                  (driveMotor.getTorque(driveMotor.getCurrent(0, angularkA.get()))
                          * TunerConstants.FrontLeft.DriveMotorGearRatio
                          / TunerConstants.FrontLeft.WheelRadius)
                      * driveRadius.in(Meters)
                      * 4;
              double MOI = chassisTorque / 1.0;

              FaultLogger.report("Chassis MOI: " + MOI, FaultType.INFO);
            })
        .ignoringDisable(true)
        .withName("Calculate Chassis MOI");
  }

  /** Calculates the drive wheel coefficient of static friction. */
  public Command calculateWheelCOF() {
    return Commands.runOnce(
            () -> {
              DCMotor driveMotor = DCMotor.getKrakenX60(1);

              double totalFrictionForce =
                  (driveMotor.getTorque(TunerConstants.FrontLeft.SlipCurrent)
                          * TunerConstants.FrontLeft.DriveMotorGearRatio
                          / TunerConstants.FrontLeft.WheelRadius)
                      * 4;

              double cof = totalFrictionForce / (SwerveConstants.mass.in(Kilograms) * 9.81);

              FaultLogger.report("Drive Wheel COF: " + cof, FaultType.INFO);
            })
        .ignoringDisable(true)
        .withName("Calculate Wheel COF");
  }

  /**
   * Calculate the drive motor max speed and torque accounting for pose PID headroom and stator
   * current limit.
   */
  public Command calculateMotorMaxSpeedAndTorque() {
    // percentage of max achievable speed and torque, leaving headroom for pose PID
    final double headroom = 0.80;

    return Commands.runOnce(
            () -> {
              DCMotor driveMotor = DCMotor.getKrakenX60(1);

              double maxSpeed =
                  Units.radiansPerSecondToRotationsPerMinute(
                      driveMotor.freeSpeedRadPerSec * headroom);
              double maxTorque =
                  driveMotor.getTorque(TunerConstants.FrontLeft.SlipCurrent) * headroom;

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

  // TODO: add self check routines
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
