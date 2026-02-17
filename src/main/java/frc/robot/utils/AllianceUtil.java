package frc.robot.utils;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShotConstants;

public class AllianceUtil {
  private static final ShotParameters _shotParameters = new ShotParameters();

  // fpi constants
  private static final int maxIter = 10;
  private static final double dTtolerance = 0.01;
  private static final double contractionRate = 0.8;

  /** Gets the alliance from the DS. If the alliance can't be retreived, blue is used by default. */
  public static Alliance getAlliance() {
    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent()) {
      return alliance.get();
    }

    return Alliance.Blue;
  }

  /** Whether the supplied robot pose is in the ferry zone, depending on alliance. */
  public static boolean inFerryZone(Pose2d robotPose) {
    if (getAlliance() == Alliance.Blue) {
      return robotPose.getX() > FieldConstants.ferryXThresholdBlue;
    }

    return robotPose.getX() < FieldConstants.ferryXThresholdRed;
  }

  /** Gets the correct ferry target location for shooting. */
  public static Translation2d getFerryTarget(Pose2d robotPose) {
    if (getAlliance() == Alliance.Blue) {
      return robotPose.getY() > FieldConstants.ferryYThreshold
          ? FieldConstants.blueFerryTop
          : FieldConstants.blueFerryBottom;
    }

    return robotPose.getY() > FieldConstants.ferryYThreshold
        ? FieldConstants.redFerryTop
        : FieldConstants.redFerryBottom;
  }

  /** Gets the correct hub target location for shooting. */
  public static Translation2d getHubTarget() {
    return getAlliance() == Alliance.Blue ? FieldConstants.blueHub : FieldConstants.redHub;
  }

  /** Using the robot pose, finds the shot target location (hub / ferry). */
  public static Translation2d getShotTarget(Pose2d robotPose) {
    return inFerryZone(robotPose) ? getFerryTarget(robotPose) : getHubTarget();
  }

  /**
   * Finds the shot target at the current robot pose, then uses fixed-point iteration to find the
   * correct shot parameters for the robot speeds.
   */
  public static ShotParameters getShotParameters(Pose2d robotPose, ChassisSpeeds robotSpeeds) {
    InterpolatingMatrixTreeMap<Double, N4, N1> presets =
        inFerryZone(robotPose) ? ShotConstants.ferryPresets : ShotConstants.hubPresets;

    InterpolatingDoubleTreeMap TOFs =
        inFerryZone(robotPose) ? ShotConstants.ferryTOFs : ShotConstants.hubTOFs;

    Translation2d target = getShotTarget(robotPose);

    double t = 0;
    double prev_t = 0;

    Translation2d robotSpeedsVec =
        new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);

    // // fixed-point iteration
    // for (int i = 0; i < maxIter; i++) {
    //   // t_n+1 = T(t_n)
    //   double virtualTargetDistance =
    //       target.minus(robotSpeedsVec.times(t)).getDistance(robotPose.getTranslation());
    //   double new_t = TOFs.get(virtualTargetDistance);

    //   double dT_dt = (t != prev_t) ? (new_t - t) / (t - prev_t) : 0; // prevent division by 0

    //   prev_t = t;
    //   t = new_t;

    //   if (Math.abs(dT_dt) > contractionRate) {
    //     _shotParameters.isValid = false;
    //     _shotParameters.fpiIterations = i + 1;

    //     break;
    //   }

    //   if (Math.abs(t - prev_t) < dTtolerance) {
    //     Translation2d virtualTarget = target.minus(robotSpeedsVec.times(t));

    //     _shotParameters.setPreset(
    //         presets.get(virtualTarget.getDistance(robotPose.getTranslation())));
    //
    // _shotParameters.setShotHeading(virtualTarget.minus(robotPose.getTranslation()).getAngle());
    //     _shotParameters.setVirtualTarget(virtualTarget);
    //     _shotParameters.fpiIterations = i + 1;

    //     break;
    //   }
    // }

    // newton
    Translation2d virtualTargetDisplacementVec =
        target.minus(robotPose.getTranslation()).minus(robotSpeedsVec.times(t));
    double intialProjectileVelocity =
        virtualTargetDisplacementVec.getNorm() / TOFs.get(virtualTargetDisplacementVec.getNorm());

    _shotParameters.projectileVelocity = intialProjectileVelocity;

    for (int i = 0; i < maxIter; i++) {
      double E = t - (virtualTargetDisplacementVec.getNorm() / intialProjectileVelocity);
      double dE =
          1
              + (virtualTargetDisplacementVec.dot(robotSpeedsVec)
                  / (virtualTargetDisplacementVec.getNorm() * intialProjectileVelocity));

      double new_t = t - (E / dE);

      if (Math.abs(new_t - t) < dTtolerance) {
        Translation2d virtualTarget = target.minus(robotSpeedsVec.times(new_t));

        _shotParameters.setPreset(
            presets.get(virtualTarget.getDistance(robotPose.getTranslation())));
        _shotParameters.setShotHeading(virtualTarget.minus(robotPose.getTranslation()).getAngle());
        _shotParameters.setVirtualTarget(virtualTarget);
        _shotParameters.fpiIterations = i + 1;

        break;
      }

      t = new_t;
      virtualTargetDisplacementVec =
          target.minus(robotPose.getTranslation()).minus(robotSpeedsVec.times(t));
    }

    return _shotParameters;
  }
}
