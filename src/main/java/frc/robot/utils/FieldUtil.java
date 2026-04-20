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
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShotConstants;

public class FieldUtil {
  // newton's method constants
  private static final int maxIter = 15;
  private static final double E_tolerance = 0.1;
  private static final double couplingDegreesTolerance = 20;

  private static final double robotVelocityNormTolerance = 0.1;

  private static final double hubActiveLeadTime = 1.0;

  private static boolean _prevHubActive;
  private static double _prevShiftTime = -1;

  private static final BooleanSubscriber _useFudge = DogLog.tunable("Use Fudge", true);

  static {
    // teleop().onTrue(runOnce(() -> _useFudge.getTopic().publish().set(true)));
  }

  /** Logs FieldUtil methods. */
  public static void log(Pose2d robotPose) {
    DogLog.log("FieldUtil/Alliance", getAlliance());
    DogLog.log("FieldUtil/Is Hub Active", isHubActive());
    DogLog.log("FieldUtil/Is Shot Valid", isShotValid(robotPose));
    DogLog.log("FieldUtil/In Bump Zone", inBumpZone(robotPose));
    DogLog.log("FieldUtil/In Alliance Zone", inAllianceZone(robotPose));
    DogLog.log("FieldUtil/Match Time", getMatchTime());
    DogLog.log("FieldUtil/Shift Time", getShiftTime());
  }

  /** Gets the alliance from the DS. If the alliance can't be retreived, blue is used by default. */
  public static Alliance getAlliance() {
    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent()) {
      return alliance.get();
    }

    return Alliance.Blue;
  }

  /** Checks whether this alliance's hub is active. */
  public static boolean isHubActive() {
    if (DriverStation.isAutonomous()) {
      return true;
    }

    String data = DriverStation.getGameSpecificMessage();
    if (data.isEmpty()) return true;

    boolean active = false;
    double time = getMatchTime();

    if (time >= 130 || time <= 30 + hubActiveLeadTime) return true;

    switch (getAlliance()) {
      case Blue -> active = data.charAt(0) != 'B';
      case Red -> active = data.charAt(0) != 'R';
    }

    int flips = (int) ((130 - time) / 25.0);
    int futureFlips = (int) ((130 - (time - hubActiveLeadTime)) / 25.0);
    int activeParity = active ? 0 : 1;

    if (flips % 2 == 1) active = !active;

    if (!active && futureFlips % 2 == activeParity) active = !active;

    return active;
  }

  /** Gets approximate match time from Driver Station. */
  public static double getMatchTime() {
    return DriverStation.getMatchTime();
  }

  /** Gets the shift time for when each alliance hub is active. */
  public static double getShiftTime() {
    double time = getMatchTime();

    if (time > 130) return time - 130;
    if (time <= (30 + hubActiveLeadTime)) return time;

    boolean current = isHubActive();

    if (_prevShiftTime == -1) {
      _prevHubActive = current;
      _prevShiftTime = 130 + (_prevHubActive ? hubActiveLeadTime : 0);
    }

    if (current != _prevHubActive) {
      _prevShiftTime = time;
      _prevHubActive = current;
    }

    double elapsed = _prevShiftTime - time;
    return current ? (25 + hubActiveLeadTime) - elapsed : (25 - hubActiveLeadTime) - elapsed;
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

    shotParameters.setTarget(target.getX(), target.getY());

    double robotVelocityX = robotSpeeds.vxMetersPerSecond;
    double robotVelocityY = robotSpeeds.vyMetersPerSecond;

    double robotVelocityNorm = Math.hypot(robotVelocityX, robotVelocityY);

    // treat small robot velocity as 0
    if (robotVelocityNorm < robotVelocityNormTolerance) {
      robotVelocityX = 0;
      robotVelocityY = 0;

      robotVelocityNorm = 0;
    }

    double robotToVirtualTargetX = target.getX() - robotPose.getX();
    double robotToVirtualTargetY = target.getY() - robotPose.getY();

    double robotToVirtualTargetNorm = Math.hypot(robotToVirtualTargetX, robotToVirtualTargetY);

    double robotToVirtualTarget_dot_robotVelocity =
        robotToVirtualTargetX * robotVelocityX + robotToVirtualTargetY * robotVelocityY;

    // initial guess for t
    double t =
        robotToVirtualTargetNorm
            / (robotToVirtualTarget_dot_robotVelocity / robotToVirtualTargetNorm
                + projectileHorizontalVelocity);

    for (int i = 0; i < maxIter; i++) {
      robotToVirtualTargetX = target.getX() - robotPose.getX() - robotVelocityX * t;
      robotToVirtualTargetY = target.getY() - robotPose.getY() - robotVelocityY * t;

      robotToVirtualTargetNorm = Math.hypot(robotToVirtualTargetX, robotToVirtualTargetY);

      robotToVirtualTarget_dot_robotVelocity =
          robotToVirtualTargetX * robotVelocityX + robotToVirtualTargetY * robotVelocityY;

      double T = TOFs.get(robotToVirtualTargetNorm);
      double dT_dt = 0;

      // if D is within LUT
      if (robotToVirtualTargetNorm
          == MathUtil.clamp(robotToVirtualTargetNorm, minDistance, maxDistance)) {
        dT_dt =
            -robotToVirtualTarget_dot_robotVelocity
                / (projectileHorizontalVelocity * robotToVirtualTargetNorm);
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
                        robotToVirtualTarget_dot_robotVelocity
                            / (robotToVirtualTargetNorm * robotVelocityNorm))));

        shotParameters.isErrorSensitive = shotParameters.couplingDegrees < couplingDegreesTolerance;

        shotParameters.setPreset(
            presets.get(
                robotToVirtualTargetNorm
                    + (_useFudge.get() ? ShotConstants.distanceToTargetFudge.in(Meters) : 0)));
        shotParameters.setShotHeading(Math.atan2(robotToVirtualTargetY, robotToVirtualTargetX));

        double virtualTargetX = target.getX() - robotVelocityX * t;
        double virtualTargetY = target.getY() - robotVelocityY * t;

        shotParameters.setVirtualTarget(virtualTargetX, virtualTargetY);

        shotParameters.newtonIterations = i + 1;
        shotParameters.failedToConverge = false;

        // check if shot is in bounds
        shotParameters.inBounds =
            minDistance <= robotToVirtualTargetNorm && robotToVirtualTargetNorm <= maxDistance;

        return;
      }

      t = t - (E / dE_dt);
    }

    shotParameters.failedToConverge = true;
  }
}
