package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.utility.WheelForceCalculator;
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;
import dev.doglog.DogLog;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Robot;

public class HolonomicController {
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
  private double _goalHeading = 0;

  private final PIDController _xController =
      new PIDController(SwerveConstants.poseTranslationalkP.in(MetersPerSecond.per(Meter)), 0, 0);
  private final PIDController _yController =
      new PIDController(SwerveConstants.poseTranslationalkP.in(MetersPerSecond.per(Meter)), 0, 0);

  private final PIDController _headingController =
      new PIDController(SwerveConstants.poseRotationkP.in(RadiansPerSecond.per(Radian)), 0, 0);

  private final WheelForceCalculator _wheelForceCalculator;
  private Feedforwards _wheelForces = new Feedforwards(4);

  private ChassisSpeeds _prevSetpointSpeeds = new ChassisSpeeds();

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

  /** The wheel forces based on the acceleration of the profiles. */
  public Feedforwards getWheelForces() {
    return _wheelForces;
  }

  /** Whether the profiles have been completed or not. */
  public boolean isFinished() {
    return _translationProfile.getSetpoint().equals(_translationProfile.getGoal())
        && _headingProfile.getSetpoint().equals(_headingProfile.getGoal());
  }

  /** Resets the PID controllers. */
  public void reset() {
    _xController.reset();
    _yController.reset();

    _headingController.reset();
  }

  /**
   * Resets the profiles and the PID controllers.
   *
   * @param currentPose The current pose.
   * @param goalPose The goal pose.
   * @param currentSpeeds The current field-relative speeds of the chassis.
   */
  public void reset(Pose2d currentPose, Pose2d goalPose, ChassisSpeeds currentSpeeds) {
    if (!SwerveConstants.ignorePoseTolerance) {
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

    _translationProfile.reset(
        0,
        _translationDirection.dot(
                VecBuilder.fill(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond))
            / _translationDirection.norm());
    _headingProfile.reset(
        currentPose.getRotation().getRadians(), currentSpeeds.omegaRadiansPerSecond);

    _prevSetpointSpeeds = currentSpeeds;

    reset();

    _startPose = currentPose;
    _goalHeading = goalPose.getRotation().getRadians();
  }

  /**
   * Samples the motions profiles at the next timestep, finding chassis speeds based on the profiles
   * and PID correction.
   *
   * @param currentPose The current pose.
   * @return Field-relative speeds for the chassis.
   */
  public ChassisSpeeds calculate(Pose2d currentPose) {
    _translationProfile.calculate(
        0,
        _translationDirection.norm()); // measurement doesn't matter, handled by xy PID controllers
    _headingProfile.calculate(
        currentPose.getRotation().getRadians(), _goalHeading); // TODO: why does measurement matter?

    Vector<N2> setpointPosition =
        _translationDirection.unit().times(_translationProfile.getSetpoint().position);
    Vector<N2> setpointVelocity =
        _translationDirection.unit().times(_translationProfile.getSetpoint().velocity);

    Pose2d setpointPose =
        new Pose2d(
            _startPose.getX() + setpointPosition.get(0),
            _startPose.getY() + setpointPosition.get(1),
            new Rotation2d(_headingProfile.getSetpoint().position));

    ChassisSpeeds setpointSpeeds =
        new ChassisSpeeds(
            setpointVelocity.get(0),
            setpointVelocity.get(1),
            _headingProfile.getSetpoint().velocity);

    _wheelForces =
        _wheelForceCalculator.calculate(Robot.kDefaultPeriod, _prevSetpointSpeeds, setpointSpeeds);

    _prevSetpointSpeeds = setpointSpeeds;

    return calculate(setpointSpeeds, setpointPose, currentPose);
  }

  /**
   * Modifies some base field-relative chassis speeds the chassis is currently traveling at to bring
   * it closer to the desired pose.
   *
   * @param baseSpeeds The field-relative speed the chassis is already traveling at.
   * @param desiredPose The desired pose.
   * @param currentPose The current pose.
   * @return New modified field-relative speeds.
   */
  public ChassisSpeeds calculate(ChassisSpeeds baseSpeeds, Pose2d desiredPose, Pose2d currentPose) {
    DogLog.log("Auto/Controller Desired Pose", desiredPose);
    DogLog.log("Auto/Controller Reference Pose", currentPose);

    if (!SwerveConstants.ignorePoseTolerance) {
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

      return baseSpeeds.plus(new ChassisSpeeds(velX, velY, velOmega));
    }

    return baseSpeeds.plus(
        new ChassisSpeeds(
            _xController.calculate(currentPose.getX(), desiredPose.getX()),
            _yController.calculate(currentPose.getY(), desiredPose.getY()),
            _headingController.calculate(
                currentPose.getRotation().getRadians(), desiredPose.getRotation().getRadians())));
  }
}
