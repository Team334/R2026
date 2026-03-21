// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.utils.ShotParameters.vec3;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.VisionPoseEstimator.VisionPoseEstimatorConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // temporary sim can fixes
  private static final CANBus determineBus(String bus) {
    if (Robot.isSimulation()) {
      return new CANBus("");
    }

    return new CANBus(bus);
  }

  private static final int determineID(int id) {
    if (Robot.isSimulation()) {
      return 20 + id;
    }

    return id;
  }

  public static final Frequency simNotifierFrequency = Hertz.of(200);

  public static final CANBus subsystemBus = determineBus("subsystems");

  public static class Ports {
    public static final int driverController = 0;
  }

  public static class MotorConstants {
    public static final DCMotor krakenX60 =
        new DCMotor(12, 7.16, 374.4, 3, Units.rotationsPerMinuteToRadiansPerSecond(6065), 1);

    public static final DCMotor krakenX44 =
        new DCMotor(12, 4.11, 279.1, 3, Units.rotationsPerMinuteToRadiansPerSecond(7758), 1);
  }

  public static class FieldConstants {
    public static final AprilTagFieldLayout tagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public static final Translation2d blueHub =
        new Translation2d(
            tagLayout.getTagPose(26).get().getX() + Units.inchesToMeters(47.0) / 2.0,
            tagLayout.getFieldWidth() / 2.0);

    public static final Translation2d redHub =
        blueHub.rotateAround(
            new Translation2d(tagLayout.getFieldLength() / 2.0, tagLayout.getFieldWidth() / 2.0),
            Rotation2d.k180deg);

    public static final Translation2d blueFerryBottom = new Translation2d(2, 2);
    public static final Translation2d blueFerryTop =
        new Translation2d(2, tagLayout.getFieldWidth() - blueFerryBottom.getY());

    public static final Translation2d redFerryBottom =
        new Translation2d(
            tagLayout.getFieldLength() - blueFerryBottom.getX(), blueFerryBottom.getY());
    public static final Translation2d redFerryTop =
        new Translation2d(redFerryBottom.getX(), blueFerryTop.getY());

    public static final double ferryXThresholdBlue = 5;
    public static final double ferryXThresholdRed =
        tagLayout.getFieldLength() - ferryXThresholdBlue;

    public static final double ferryYThreshold = tagLayout.getFieldWidth() / 2.0;

    public static final Rectangle2d blueBumpZone = new Rectangle2d(Pose2d.kZero, 1, 1);
    public static final Rectangle2d redBumpZone = new Rectangle2d(Pose2d.kZero, 1, 1);

    public static final Pose2d bluePreClimb = new Pose2d();
    public static final Pose2d redPreClimb = new Pose2d();

    public static final Pose2d blueClimb = new Pose2d();
    public static final Pose2d redClimb = new Pose2d();

    // uncomment if using the test tag layout
    // public static final AprilTagFieldLayout tagLayout;

    // static {
    //   try {
    //     tagLayout =
    //         new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/test-tag-layout.json");
    //   } catch (Exception e) {
    //     throw new RuntimeException(e);
    //   }
    // }
  }

  public static class VisionConstants {
    public static final double singleTagStdDevsScaler = 5;

    public static final double ambiguityThreshold = 0.2;

    public static final double xBoundMargin = 0.01;
    public static final double yBoundMargin = 0.01;
    public static final double zBoundMargin = 0.1;

    public static final String leftArducamName = "left-arducam";
    public static final String rightArducamName = "right-arducam";

    public static final VisionPoseEstimatorConstants leftArducam =
        new VisionPoseEstimatorConstants(
            leftArducamName,
            new Transform3d(
                new Translation3d(
                    -Units.inchesToMeters(9.8578),
                    Units.inchesToMeters(9.6913),
                    Units.inchesToMeters(20.2395)),
                new Rotation3d(0, 0, -Units.degreesToRadians(15))),
            0.1,
            3,
            7);

    public static final VisionPoseEstimatorConstants rightArducam =
        new VisionPoseEstimatorConstants(
            rightArducamName,
            new Transform3d(
                new Translation3d(
                    -Units.inchesToMeters(9.8578),
                    -Units.inchesToMeters(9.6913),
                    Units.inchesToMeters(20.2395)),
                new Rotation3d(0, 0, Units.degreesToRadians(15))),
            0.1,
            3,
            7);
  }

  public static class ShotConstants {
    public static final AngularVelocity spitFlywheelSpeed = RotationsPerSecond.of(5);

    public static final AngularVelocity spitRollerSpeed = RotationsPerSecond.of(1);
    public static final AngularVelocity spitFloorSpeed = RotationsPerSecond.of(1);

    public static final AngularVelocity flywheelVelocityThreshold = RotationsPerSecond.of(3);

    // distanceMeters : <flywheelRPS, rollerRPS, floorRPS>
    public static InterpolatingMatrixTreeMap<Double, N3, N1> hubPresets =
        new InterpolatingMatrixTreeMap<>();
    public static InterpolatingMatrixTreeMap<Double, N3, N1> ferryPresets =
        new InterpolatingMatrixTreeMap<>();

    public static final Distance hubMinDistance = Meters.of(1.89);
    public static final Distance hubMaxDistance = Meters.of(5.252);

    public static final Distance ferryMinDistance = Meters.of(2.767);
    public static final Distance ferryMaxDistance = Meters.of(4.0);

    // distanceMeters : TOFSecs
    public static InterpolatingDoubleTreeMap hubTOFs = new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap ferryTOFs = new InterpolatingDoubleTreeMap();

    static {
      // hub presets
      hubPresets.put(1.89, vec3(38, 50, 40));
      hubPresets.put(2.665, vec3(43, 50, 40));
      hubPresets.put(3.768, vec3(47.75, 50, 40));
      hubPresets.put(4.574, vec3(51.8, 50, 40));
      hubPresets.put(5.252, vec3(52, 50, 40));

      // ferry presets
      ferryPresets.put(0.0, vec3(0, 0, 0));

      // hub TOFs
      hubTOFs.put(1.89, 0.955);
      hubTOFs.put(2.665, 1.08);
      hubTOFs.put(3.768, 1.38);
      hubTOFs.put(4.574, 1.53);
      hubTOFs.put(5.252, 1.525);

      // ferry TOFs
      ferryTOFs.put(0.0, 0.00000001);
    }
  }

  public static class ShooterConstants {
    public static final int flywheelMotorID = determineID(1); // 1
    public static final int flywheelFollowerMotorID = determineID(2); // 2

    public static final Voltage flywheelkS = Volts.of(0.13262);
    public static final Per<VoltageUnit, AngularVelocityUnit> flywheelkV =
        Volts.per(RotationsPerSecond).ofNative(0.13334);
    public static final Per<VoltageUnit, AngularVelocityUnit> flywheelkP =
        Volts.per(RotationsPerSecond).ofNative(0.2);

    public static final double flywheelGearRatio = 1.2;
  }

  public static class HopperConstants {
    public static final int rollerMotorID = determineID(3);
    public static final int floorMotorID = determineID(4);

    public static final Voltage rollerkS = Volts.of(0.17);
    public static final Per<VoltageUnit, AngularVelocityUnit> rollerkV =
        Volts.per(RotationsPerSecond).ofNative(0.235);
    public static final Per<VoltageUnit, AngularVelocityUnit> rollerkP =
        Volts.per(RotationsPerSecond).ofNative(0.6);

    public static final Voltage floorkS = Volts.of(0.21);
    public static final Per<VoltageUnit, AngularVelocityUnit> floorkV =
        Volts.per(RotationsPerSecond).ofNative(0.28);
    public static final Per<VoltageUnit, AngularVelocityUnit> floorkP =
        Volts.per(RotationsPerSecond).ofNative(0.6);

    public static final double rollerGearRatio = 2.25;
    public static final double floorGearRatio = 2;
  }

  public static class IntakeConstants {
    public static final int pivotMotorID = determineID(5);
    public static final int feedMotorID = determineID(6);

    public static final Voltage feedkS = Volts.of(0.34);

    public static final Per<VoltageUnit, AngularVelocityUnit> feedkV =
        Volts.per(RotationsPerSecond).ofNative(0.205);

    public static final Per<VoltageUnit, AngularVelocityUnit> feedkP =
        Volts.per(RotationsPerSecond).ofNative(0.6);

    public static final Voltage pivotkG = Volts.of(0);
    public static final Voltage pivotkS = Volts.of(0);

    public static final Per<VoltageUnit, AngularVelocityUnit> pivotkV =
        Volts.per(RotationsPerSecond).ofNative(0);
    public static final Per<VoltageUnit, AngularAccelerationUnit> pivotkA =
        Volts.per(RotationsPerSecondPerSecond).ofNative(0);

    public static final Per<VoltageUnit, AngleUnit> pivotkP = Volts.per(Rotations).ofNative(0);

    public static final AngularVelocity pivotVelocity = RotationsPerSecond.of(2);
    public static final AngularAcceleration pivotAcceleration = RotationsPerSecondPerSecond.of(5);

    public static final double feedGearRatio = 2;
    public static final double pivotGearRatio = 20;

    public static final Angle pivotRaised = Rotations.of(0.25);
    public static final Angle pivotTucked = Rotations.of(0.5);
    public static final Angle pivotLowered = Rotations.of(0.6);

    public static final Angle pivotForwardSoftLimitThreshold = Rotations.of(0.7);
    public static final Angle pivotReverseSoftLimitThreshold = Rotations.of(0.2);

    public static final AngularVelocity feedSpeed = RotationsPerSecond.of(30);
  }

  public static class ClimbConstants {
    public static final int climbMotorID = determineID(7);

    public static final Voltage kS = Volts.of(0);
    public static final Voltage kG = Volts.of(0);
    public static final Per<VoltageUnit, AngularVelocityUnit> kV =
        Volts.per(RotationsPerSecond).ofNative(0);
    public static final Per<VoltageUnit, AngularAccelerationUnit> kA =
        Volts.per(RotationsPerSecondPerSecond).ofNative(0);

    public static final Per<VoltageUnit, AngleUnit> kP = Volts.per(Rotations).ofNative(0);

    public static final Voltage climbingkG = Volts.of(0);
    public static final Per<VoltageUnit, AngularAccelerationUnit> climbingkA =
        Volts.per(RotationsPerSecondPerSecond).ofNative(0);
    public static final Per<VoltageUnit, AngleUnit> climbingkP = Volts.per(Rotations).ofNative(0);

    public static final Angle retracted = Rotations.of(0);
    public static final Angle extended = Rotations.of(10);

    public static final Angle extendedTolerance = Rotations.of(0.5);

    public static final Angle forwardSoftLimitThreshold = Rotations.of(11);
    public static final Angle reverseSoftLimitThreshold = Rotations.of(-1);

    public static final double climbGearRatio = 3;
  }

  public static class SwerveConstants {
    public static final Frequency odometryFrequency = Hertz.of(250);

    public static final Mass mass = Pounds.of(103);
    public static final MomentOfInertia moi = KilogramSquareMeters.of(0);

    public static final LinearVelocity driverTranslationalVelocity = MetersPerSecond.of(4);
    public static final AngularVelocity driverAngularVelocity = RadiansPerSecond.of(Math.PI);

    public static final LinearVelocity driverTranslationalShootingVelocity =
        MetersPerSecond.of(1.5);

    public static final LinearVelocity profileTranslationalVelocity = MetersPerSecond.of(1);
    public static final LinearAcceleration profileTranslationalAcceleration =
        MetersPerSecondPerSecond.of(2);

    public static final AngularVelocity profileAngularVelocity = RadiansPerSecond.of(Math.PI * 2);
    public static final AngularAcceleration profileAngularAcceleration =
        RadiansPerSecondPerSecond.of(4);

    public static final Per<LinearVelocityUnit, DistanceUnit> poseTranslationalkP =
        MetersPerSecond.per(Meter).ofNative(2);
    public static final Per<LinearVelocityUnit, LinearVelocityUnit> poseTranslationalkD =
        MetersPerSecond.per(MetersPerSecond).ofNative(0);

    public static final Per<AngularVelocityUnit, AngleUnit> poseRotationkP =
        RadiansPerSecond.per(Radian).ofNative(4.5);
    public static final Per<AngularVelocityUnit, AngularVelocityUnit> poseRotationkD =
        RadiansPerSecond.per(RadiansPerSecond).ofNative(0);

    public static final boolean ignorePoseTolerance = true;

    public static final Translation2d poseTranslationTolerance = new Translation2d(0.03, 0.03);
    public static final Rotation2d poseRotationTolerance = Rotation2d.fromDegrees(1);

    public static LinearVelocity translationalDeadband = MetersPerSecond.of(0.01);
    public static AngularVelocity rotationalDeadband = RadiansPerSecond.of(0.01);
  }
}
