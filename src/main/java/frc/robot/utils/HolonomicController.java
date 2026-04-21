package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.utility.WheelForceCalculator;
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;
import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.lib.math.FilteredPIDController;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class HolonomicController {
  // pid part of these controllers is UNUSED, this class is just used since it already
  // manages updating the profile setpoint and goal internally
  private final ProfiledPIDController _translationProfile =
      new ProfiledPIDController(
          0,
          0,
          0,
          new Constraints(
              SwerveConstants.profileTranslationalVelocity.in(MetersPerSecond),
              SwerveConstants.profileTranslationalAcceleration.in(MetersPerSecondPerSecond)));

  private final ProfiledPIDController _headingProfile =
      new ProfiledPIDController(
          0,
          0,
          0,
          new Constraints(
              SwerveConstants.profileAngularVelocity.in(RadiansPerSecond),
              SwerveConstants.profileAngularAcceleration.in(RadiansPerSecondPerSecond)));

  private Vector<N2> _translationDirection = VecBuilder.fill(0, 0);

  private Pose2d _startPose = Pose2d.kZero;

  private final FilteredPIDController _xController =
      new FilteredPIDController(
          SwerveConstants.poseTranslationalkP.in(MetersPerSecond.per(Meter)),
          SwerveConstants.poseTranslationalkD.in(MetersPerSecond.per(MetersPerSecond)),
          Constants.robotPeriod.in(Seconds),
          0.2);
  private final FilteredPIDController _yController =
      new FilteredPIDController(
          SwerveConstants.poseTranslationalkP.in(MetersPerSecond.per(Meter)),
          SwerveConstants.poseTranslationalkD.in(MetersPerSecond.per(MetersPerSecond)),
          Constants.robotPeriod.in(Seconds),
          0.2);

  private final FilteredPIDController _headingController =
      new FilteredPIDController(
          SwerveConstants.poseRotationkP.in(RadiansPerSecond.per(Radian)),
          SwerveConstants.poseRotationkD.in(RadiansPerSecond.per(RadiansPerSecond)),
          Constants.robotPeriod.in(Seconds),
          0.2);

  private ChassisSpeeds _pidSpeeds = new ChassisSpeeds();

  private final WheelForceCalculator _wheelForceCalculator;
  private Feedforwards _wheelForces = new Feedforwards(4);

  private ChassisSpeeds _setpointSpeeds = new ChassisSpeeds();
  private Pose2d _setpointPose = Pose2d.kZero;

  private ChassisSpeeds _prevSetpointSpeeds = new ChassisSpeeds();

  private final ChassisSpeeds zeroSpeeds = new ChassisSpeeds();

  /**
   * Creates a new HolonomicController.
   *
   * @param moduleLocations The locations of the modules.
   */
  public HolonomicController(Translation2d[] moduleLocations) {
    _headingController.enableContinuousInput(-Math.PI, Math.PI);
    _headingProfile.enableContinuousInput(-Math.PI, Math.PI);

    _wheelForceCalculator =
        new WheelForceCalculator(moduleLocations, SwerveConstants.mass, SwerveConstants.moi);
  }

  /** Whether r'(t) on the translation PID controllers is being filtered. */
  @Logged(name = "Is Filtering Translation")
  public boolean isFilteringTranslation() {
    return _xController.isFilteringSetpointVelocity() && _yController.isFilteringSetpointVelocity();
  }

  /** Sets r'(t) low-pass filtering on the translation pid controllers. */
  public void useFilteringTranslation(boolean use) {
    if (use) {
      _xController.enableSetpointVelocityFilter();
      _yController.enableSetpointVelocityFilter();
    } else {
      _xController.disableSetpointVelocityFilter();
      _yController.disableSetpointVelocityFilter();
    }
  }

  /** Whether r'(t) on the heading PID controller is being filtered. */
  @Logged(name = "Is Filtering Heading")
  public boolean isFilteringHeading() {
    return _headingController.isFilteringSetpointVelocity();
  }

  /** Sets r'(t) low-pass filtering on the heading pid controller. */
  public void useFilteringHeading(boolean use) {
    if (use) {
      _headingController.enableSetpointVelocityFilter();
    } else {
      _headingController.disableSetpointVelocityFilter();
    }
  }

  /** Wheel force calculator for any external calculations. */
  public WheelForceCalculator getWheelForceCalculator() {
    return _wheelForceCalculator;
  }

  /** The wheel forces based on the acceleration of the profiles. */
  public Feedforwards getWheelForces() {
    return _wheelForces;
  }

  /** Profile setpoint speeds. */
  public ChassisSpeeds getSetpointSpeeds() {
    return _setpointSpeeds;
  }

  /** Profile setpoint pose. */
  public Pose2d getSetpointPose() {
    return _setpointPose;
  }

  /** Whether the profiles have been completed or not. */
  public boolean isFinished() {
    return _translationProfile.getSetpoint().equals(_translationProfile.getGoal())
        && _headingProfile.getSetpoint().equals(_headingProfile.getGoal());
  }

  /**
   * Resets the profiles.
   *
   * @param currentPose The current pose.
   * @param goalPose The goal pose.
   * @param currentSpeeds The current field-relative speeds of the chassis.
   */
  public void reset(Pose2d currentPose, Pose2d goalPose, ChassisSpeeds currentSpeeds) {
    if (!SwerveConstants.ignorePoseTolerance) {
      // if the goal pose is very close to current pose on a certain axis, don't move on that axis
      Translation2d translationError =
          goalPose.getTranslation().minus(currentPose.getTranslation());
      Rotation2d rotationError = goalPose.getRotation().minus(currentPose.getRotation());

      double goalX =
          Math.abs(translationError.getX()) <= SwerveConstants.poseTranslationTolerance.getX()
              ? currentPose.getX()
              : goalPose.getX();

      double goalY =
          Math.abs(translationError.getY()) <= SwerveConstants.poseTranslationTolerance.getY()
              ? currentPose.getY()
              : goalPose.getY();

      double goalHeading =
          Math.abs(rotationError.getRadians()) <= SwerveConstants.poseRotationTolerance.getRadians()
              ? currentPose.getRotation().getRadians()
              : goalPose.getRotation().getRadians();

      goalPose = new Pose2d(goalX, goalY, Rotation2d.fromRadians(goalHeading));
    }

    _translationDirection =
        VecBuilder.fill(goalPose.getX() - currentPose.getX(), goalPose.getY() - currentPose.getY());

    // set profile goals
    _translationProfile.setGoal(_translationDirection.norm());
    _headingProfile.setGoal(goalPose.getRotation().getRadians());

    if (_translationDirection.norm() == 0) {
      // if the goal translation IS the starting translation:
      // 1) if current translational speeds is not 0, direction is antiparallel to speeds
      // 2) if current translational speeds is 0, profile will also be empty, so an arbitrary
      // direction is used
      _translationDirection =
          (currentSpeeds.vxMetersPerSecond == 0 && currentSpeeds.vyMetersPerSecond == 0)
              ? VecBuilder.fill(1, 0)
              : VecBuilder.fill(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond)
                  .times(-1);
    }

    // reset profile setpoints
    _translationProfile.reset(
        0,
        _translationDirection.dot(
                VecBuilder.fill(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond))
            / _translationDirection.norm());

    _headingProfile.reset(
        currentPose.getRotation().getRadians(), currentSpeeds.omegaRadiansPerSecond);

    _setpointSpeeds = currentSpeeds;
    _setpointPose = currentPose;

    _prevSetpointSpeeds = _setpointSpeeds;

    _startPose = currentPose;
  }

  /**
   * Samples the motions profiles at the next setpoint.
   *
   * @param currentHeading The current heading, needed for heading profile.
   */
  public void nextSetpoint(Rotation2d currentHeading) {
    _translationProfile.calculate(0); // measurement doesn't matter, handled by xy PID controllers
    _headingProfile.calculate(
        currentHeading.getRadians()); // measurement matters for cont input calculations

    Vector<N2> setpointPosition =
        _translationDirection.unit().times(_translationProfile.getSetpoint().position);
    Vector<N2> setpointVelocity =
        _translationDirection.unit().times(_translationProfile.getSetpoint().velocity);

    _setpointPose =
        new Pose2d(
            _startPose.getX() + setpointPosition.get(0),
            _startPose.getY() + setpointPosition.get(1),
            new Rotation2d(_headingProfile.getSetpoint().position));

    _setpointSpeeds.vxMetersPerSecond = setpointVelocity.get(0);
    _setpointSpeeds.vyMetersPerSecond = setpointVelocity.get(1);
    _setpointSpeeds.omegaRadiansPerSecond = _headingProfile.getSetpoint().velocity;

    _wheelForces =
        _wheelForceCalculator.calculate(
            Constants.robotPeriod.in(Seconds), _prevSetpointSpeeds, _setpointSpeeds);

    _prevSetpointSpeeds = _setpointSpeeds;
  }

  /**
   * Finds field-relative chassis speeds the chassis is to bring the chassis closer to the desired
   * pose.
   *
   * @param desiredPose The desired pose.
   * @param currentPose The current pose.
   * @return New field-relative speeds.
   */
  public ChassisSpeeds calculate(Pose2d desiredPose, Pose2d currentPose) {
    return calculate(zeroSpeeds, desiredPose, currentPose);
  }

  /**
   * Modifies some base field-relative chassis speeds the chassis is currently traveling at to bring
   * it closer to the desired pose.
   *
   * @param baseSpeeds The field-relative speed the chassis is already traveling at.
   * @param desiredPose The desired pose.
   * @param currentPose The current pose.
   * @return New field-relative speeds.
   */
  public ChassisSpeeds calculate(ChassisSpeeds baseSpeeds, Pose2d desiredPose, Pose2d currentPose) {
    DogLog.log("Swerve/Holonomic Controller/Controller Desired Pose", desiredPose);
    DogLog.log("Swerve/Holonomic Controller/Controller Actual Pose", currentPose);

    if (!SwerveConstants.ignorePoseTolerance) {
      // if error is very small on a certain axis, don't move on that axis
      Translation2d translationError =
          desiredPose.getTranslation().minus(currentPose.getTranslation());
      Rotation2d rotationError = desiredPose.getRotation().minus(currentPose.getRotation());

      double velX =
          Math.abs(translationError.getX()) <= SwerveConstants.poseTranslationTolerance.getX()
              ? 0
              : _xController.calculate(currentPose.getX(), desiredPose.getX());
      double velY =
          Math.abs(translationError.getY()) <= SwerveConstants.poseTranslationTolerance.getY()
              ? 0
              : _yController.calculate(currentPose.getY(), desiredPose.getY());
      double velOmega =
          Math.abs(rotationError.getRadians()) <= SwerveConstants.poseRotationTolerance.getRadians()
              ? 0
              : _headingController.calculate(
                  currentPose.getRotation().getRadians(), desiredPose.getRotation().getRadians());

      _pidSpeeds.vxMetersPerSecond = velX;
      _pidSpeeds.vyMetersPerSecond = velY;
      _pidSpeeds.omegaRadiansPerSecond = velOmega;
    } else {
      _pidSpeeds.vxMetersPerSecond = _xController.calculate(currentPose.getX(), desiredPose.getX());
      _pidSpeeds.vyMetersPerSecond = _yController.calculate(currentPose.getY(), desiredPose.getY());
      _pidSpeeds.omegaRadiansPerSecond =
          _headingController.calculate(
              currentPose.getRotation().getRadians(), desiredPose.getRotation().getRadians());
    }

    return baseSpeeds.plus(_pidSpeeds);
  }
}
