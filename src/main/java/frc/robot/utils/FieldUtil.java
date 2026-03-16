package frc.robot.utils;

import static edu.wpi.first.units.Units.MetersPerSecond;

import dev.doglog.DogLog;
import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShotConstants;
import frc.robot.Constants.SwerveConstants;

public class FieldUtil {
  // newton's method constants
  private static final int maxIter = 10;
  private static final LinearVelocity projectileHorizontalVelocity = MetersPerSecond.of(100000000);
  private static final double E_tolerance = 0.1;
  private static final double dT_dt_tolerance =
      SwerveConstants.driverTranslationalShootingVelocity.in(MetersPerSecond)
          * Math.cos(Math.toRadians(20)) // min overlap angle at worst-case robot velocity
          / projectileHorizontalVelocity.in(MetersPerSecond);

  private static double prevSwitchTime = 110; // starts at first transition shift

  /** Logs FieldUtil methods. */
  public static void log(Pose2d robotPose) {
    DogLog.log("FieldUtil/Alliance", getAlliance());
    DogLog.log("FieldUtil/Is Hub Active", isHubActive());
    DogLog.log("FieldUtil/Is Shot Valid", isShotValid(robotPose));
    DogLog.log("FieldUtil/In Bump Zone", inBumpZone(robotPose));
    DogLog.log("FieldUtil/In Alliance Zone", inAllianceZone(robotPose));
  }

  /** Gets the alliance from the DS. If the alliance can't be retreived, blue is used by default. */
  public static Alliance getAlliance() {
    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent()) {
      return alliance.get();
    }

    return Alliance.Blue;
  }

  /** Pre-climb pose based on alliance. */
  public static Pose2d getPreClimb() {
    return getAlliance() == Alliance.Blue
        ? FieldConstants.bluePreClimb
        : FieldConstants.redPreClimb;
  }

  /** Climb pose based on alliance. */
  public static Pose2d getClimb() {
    return getAlliance() == Alliance.Blue ? FieldConstants.blueClimb : FieldConstants.redClimb;
  }

  /** Checks whether this alliance's hub is active. */
  public static boolean isHubActive() {
    if (DriverStation.isAutonomous() || DriverStation.getMatchTime() <= 30) {
      return true;
    }

    String data = DriverStation.getGameSpecificMessage();
    boolean active = false;

    switch (getAlliance()) {
      case Blue:
        active = data.charAt(0) == 'B';
        break;

      case Red:
        active = data.charAt(0) == 'R';
        break;
    }

    if (prevSwitchTime - 25 >= DriverStation.getMatchTime()) {
      active = !active;
      prevSwitchTime = DriverStation.getMatchTime();
    }

    return active;
  }

  /**
   * Returns true if a shot attempted at this momement is valid. A shot is ALWAYS valid, unless it
   * is intended for the hub while the hub is inactive.
   */
  public static boolean isShotValid(Pose2d robotPose) {
    if (inAllianceZone(robotPose) && !isHubActive()) {
      return false;
    }

    return true;
  }

  /** Whether the supplied robot pose is in the bump zone(s). */
  public static boolean inBumpZone(Pose2d robotPose) {
    if (FieldConstants.blueBumpZone.contains(robotPose.getTranslation())
        || FieldConstants.redBumpZone.contains(robotPose.getTranslation())) {
      return true;
    }

    return false;
  }

  /** Whether the supplied robot pose is in the alliance zone, depending on alliance. */
  public static boolean inAllianceZone(Pose2d robotPose) {
    return !inFerryZone(robotPose);
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
    return inAllianceZone(robotPose) ? getHubTarget() : getFerryTarget(robotPose);
  }

  /**
   * Finds the shot target at the current robot pose, then uses newton's method to update the
   * supplied shot parameters.
   */
  public static void getShotParameters(
      Pose2d robotPose, ChassisSpeeds robotSpeeds, ShotParameters shotParameters) {
    InterpolatingMatrixTreeMap<Double, N3, N1> presets =
        inAllianceZone(robotPose) ? ShotConstants.hubPresets : ShotConstants.ferryPresets;

    InterpolatingDoubleTreeMap TOFs =
        inAllianceZone(robotPose) ? ShotConstants.hubTOFs : ShotConstants.ferryTOFs;

    Translation2d target = getShotTarget(robotPose);

    shotParameters.setTarget(target);

    double t = 0;

    Translation2d robotVelocity =
        new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);

    Translation2d robotToVirtualTarget =
        target.minus(robotPose.getTranslation()).minus(robotVelocity.times(t));

    for (int i = 0; i < maxIter; i++) {
      double T = TOFs.get(robotToVirtualTarget.getNorm());
      double dT_dt =
          -robotToVirtualTarget.dot(robotVelocity)
              / (projectileHorizontalVelocity.in(MetersPerSecond) * robotToVirtualTarget.getNorm());

      double E = t - T;
      double dE_dt = 1 - dT_dt;

      double new_t = t - (E / dE_dt);

      if (i == 0) {
        shotParameters.isErrorSensitive = Math.abs(dT_dt) > dT_dt_tolerance;
        shotParameters.couplingDegrees =
            Math.toDegrees(
                Math.acos(
                    Math.abs(
                        robotToVirtualTarget.dot(robotVelocity)
                            / (robotToVirtualTarget.getNorm() * robotVelocity.getNorm()))));
      }

      if (Math.abs(new_t - t) < E_tolerance) {
        Translation2d virtualTarget = target.minus(robotVelocity.times(new_t));

        shotParameters.setPreset(
            presets.get(virtualTarget.getDistance(robotPose.getTranslation())));

        shotParameters.setShotHeading(virtualTarget.minus(robotPose.getTranslation()).getAngle());
        shotParameters.setVirtualTarget(virtualTarget);

        shotParameters.newtonIterations = i + 1;

        break;
      }

      t = new_t;
      robotToVirtualTarget = target.minus(robotPose.getTranslation()).minus(robotVelocity.times(t));
    }
  }
}
