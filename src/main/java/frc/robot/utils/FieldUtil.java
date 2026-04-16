package frc.robot.utils;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.teleop;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;

public class FieldUtil {
  // newton's method constants
  private static final int maxIter = 15;
  private static final double E_tolerance = 0.1;
  private static final double couplingDegreesTolerance = 20;

  private static final double hubActiveLeadTime = 1.0;

  private static boolean prevHubActive;
  private static double prevShiftTime = -1;

  private static final BooleanSubscriber _useFudge = DogLog.tunable("Use Fudge", false);

  static {
    teleop().onTrue(runOnce(() -> _useFudge.getTopic().publish().set(true)));
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

    if (prevShiftTime == -1) {
      prevHubActive = current;
      prevShiftTime = 130 + (prevHubActive ? hubActiveLeadTime : 0);
    }

    if (current != prevHubActive) {
      prevShiftTime = time;
      prevHubActive = current;
    }

    double elapsed = prevShiftTime - time;
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

  /** Finds the shot parameters at the robot's current pose. */
  public static void getShotParameters(Pose2d robotPose, ShotParameters shotParameters) {
    // TODO
  }

  /** Finds the shot parameters at the robot's current pose. */
  // public static void getShotParameters(Pose2d robotPose, ShotParameters shotParameters) {
  //   if (shotParameters.isManual) {
  //     // unnecessary call if shot parameters are manually set
  //     return;
  //   }

  //   InterpolatingMatrixTreeMap<Double, N3, N1> presets =
  //       inAllianceZone(robotPose) ? ShotConstants.hubPresets : ShotConstants.ferryPresets;

  //   double minDistance =
  //       (inAllianceZone(robotPose) ? ShotConstants.hubMinDistance :
  // ShotConstants.ferryMinDistance)
  //           .in(Meters);
  //   double maxDistance =
  //       (inAllianceZone(robotPose) ? ShotConstants.hubMaxDistance :
  // ShotConstants.ferryMaxDistance)
  //           .in(Meters);

  //   Translation2d target = getShotTarget(robotPose);
  //   shotParameters.setTarget(target);

  //   double distanceToTarget = robotPose.getTranslation().getDistance(target);
  //   double distanceToTargetFudge =
  //       _useFudge.get() ? ShotConstants.distanceToTargetFudge.in(Meters) : 0;

  //   shotParameters.setPreset(presets.get(distanceToTarget + distanceToTargetFudge));
  //   shotParameters.setShotHeading(target.minus(robotPose.getTranslation()).getAngle());

  //   shotParameters.inBounds = minDistance <= distanceToTarget && distanceToTarget <= maxDistance;
  // }
}
