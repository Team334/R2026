package frc.robot.utils;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShotPresets;

public class AllianceUtil {
  private static final ShotPreset _shotPreset = new ShotPreset();

  private static final int maxIter = 10;
  private static final double tolerance = 0.01;
  private static final double c = 0.8;

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

  /** Shot preset depending on shooting target. */
  public static ShotPreset getShotPreset(Pose2d robotPose, ChassisSpeeds robotSpeeds) {
    InterpolatingMatrixTreeMap<Double, N2, N1> shotTable =
        inFerryZone(robotPose) ? ShotPresets.ferryTable : ShotPresets.hubTable;

    InterpolatingDoubleTreeMap TOFs = inFerryZone(robotPose) ? ShotPresets.ferryTOFs : ShotPresets.hubTOFs;

    Translation2d target = getShotTarget(robotPose);

    double t = 0;
    double prev_t = 0;

    Translation2d speeds = new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);

    for (int i = 0; i < maxIter; i++) {
      double new_t = TOFs.get(target.minus(speeds.times(t)).getDistance(robotPose.getTranslation()));
      
      double dt = (t != prev_t) ? (new_t - t) / (t - prev_t) : 0;
      
      prev_t = t;
      t = new_t;
      
      if (Math.abs(dt) > c) {
        break;
      }
      
      if (Math.abs(t - prev_t) < tolerance) {
        break;
      }
    }

    return _shotPreset.set(
        shotTable.get(shotPose.getTranslation().getDistance(getShotTarget(robotPose))));
  }
}
