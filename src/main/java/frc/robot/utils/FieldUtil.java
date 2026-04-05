package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import dev.doglog.DogLog;
import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShotConstants;
import frc.robot.Constants.SwerveConstants;

public class FieldUtil {
  // newton's method constants
  private static final int maxIter = 15;
  private static final double E_tolerance = 0.1;
  private static final double couplingDegreesTolerance = 20;

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
    double time = DriverStation.getMatchTime();

    if (DriverStation.isAutonomous() || time <= 30 || time >= 130) {
      return true;
    }

    String data = DriverStation.getGameSpecificMessage();
    if (data.isEmpty()) return true;

    boolean active = false;

    switch (getAlliance()) {
      case Blue:
        active = data.charAt(0) != 'B';
        break;

      case Red:
        active = data.charAt(0) != 'R';
        break;
    }

    int flips = (int) ((130 - time) / 25.0);

    if (flips % 2 == 1) {
      active = !active;
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
    if (shotParameters.isManual) {
      // unnecessary call if shot parameters are manually set
      return;
    }

    InterpolatingMatrixTreeMap<Double, N3, N1> presets =
        inAllianceZone(robotPose) ? ShotConstants.hubPresets : ShotConstants.ferryPresets;

    InterpolatingDoubleTreeMap TOFs =
        inAllianceZone(robotPose) ? ShotConstants.hubTOFs : ShotConstants.ferryTOFs;

    double minDistance =
        (inAllianceZone(robotPose) ? ShotConstants.hubMinDistance : ShotConstants.ferryMinDistance)
            .in(Meters);
    double maxDistance =
        (inAllianceZone(robotPose) ? ShotConstants.hubMaxDistance : ShotConstants.ferryMaxDistance)
            .in(Meters);

    double projectileHorizontalVelocity =
        (inAllianceZone(robotPose)
                ? ShotConstants.hubProjectileHorizontalVelocity
                : ShotConstants.ferryProjectileHorizontalVelocity)
            .in(MetersPerSecond);

    Translation2d target = getShotTarget(robotPose);

    shotParameters.setTarget(target);

    Translation2d robotVelocity =
        new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);

    // treat tiny robot velocity as 0
    if (robotVelocity.getNorm() < SwerveConstants.translationalDeadband.in(MetersPerSecond)) {
      robotVelocity = Translation2d.kZero;
    }

    Translation2d robotToVirtualTarget = target.minus(robotPose.getTranslation());

    // initial guess for t
    double t =
        robotToVirtualTarget.getNorm()
            / (robotToVirtualTarget.dot(robotVelocity) / robotToVirtualTarget.getNorm()
                + projectileHorizontalVelocity);

    for (int i = 0; i < maxIter; i++) {
      robotToVirtualTarget = target.minus(robotPose.getTranslation()).minus(robotVelocity.times(t));

      double T = TOFs.get(robotToVirtualTarget.getNorm());
      double dT_dt = 0;

      // if D is within LUT
      if (robotToVirtualTarget.getNorm()
          == MathUtil.clamp(robotToVirtualTarget.getNorm(), minDistance, maxDistance)) {
        dT_dt =
            -robotToVirtualTarget.dot(robotVelocity)
                / (projectileHorizontalVelocity * robotToVirtualTarget.getNorm());
      }

      double E = t - T;
      double dE_dt = 1 - dT_dt;

      // finish and update shot parameters once converged to E(t) = 0
      if (Math.abs(E) < E_tolerance) {
        // check if robot velocity vector and projectile velocity vector are strongly aligned
        shotParameters.couplingDegrees =
            Math.toDegrees(
                Math.acos(
                    Math.abs(
                        robotToVirtualTarget.dot(robotVelocity)
                            / (robotToVirtualTarget.getNorm() * robotVelocity.getNorm()))));

        shotParameters.isErrorSensitive = shotParameters.couplingDegrees < couplingDegreesTolerance;

        Translation2d virtualTarget = target.minus(robotVelocity.times(t));

        double distanceToVirtualTarget = virtualTarget.getDistance(robotPose.getTranslation());

        shotParameters.setPreset(presets.get(distanceToVirtualTarget));

        shotParameters.setShotHeading(virtualTarget.minus(robotPose.getTranslation()).getAngle());
        shotParameters.setVirtualTarget(virtualTarget);

        shotParameters.newtonIterations = i + 1;
        shotParameters.failedToConverge = false;

        // check if shot is in bounds
        shotParameters.inBounds =
            minDistance <= distanceToVirtualTarget && distanceToVirtualTarget <= maxDistance;

        return;
      }

      t = t - (E / dE_dt);
    }

    shotParameters.failedToConverge = true;
  }
}
