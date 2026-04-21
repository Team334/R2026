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
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Time;
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

  public static final Time robotPeriod = Seconds.of(0.02);
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

    public static final Rectangle2d blueBumpZone =
        new Rectangle2d(new Translation2d(4.0626, 1.5844), new Translation2d(5.1844, 6.4898));
    public static final Rectangle2d redBumpZone =
        new Rectangle2d(new Translation2d(11.3493, 1.5844), new Translation2d(12.475, 6.4898));

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
    public static final AngularVelocity towerFlywheelSpeed = RotationsPerSecond.of(53);
    public static final AngularVelocity towerRollerSpeed = RotationsPerSecond.of(50);
    public static final AngularVelocity towerFloorSpeed = RotationsPerSecond.of(50);

    public static final AngularVelocity idleSpeed = RotationsPerSecond.of(30);

    public static final AngularVelocity unjamRollerSpeed = RotationsPerSecond.of(-20);
    public static final AngularVelocity unjamFloorSpeed = RotationsPerSecond.of(-20);

    public static final Distance distanceToTargetFudge = Meters.of(-0.2);

    // distanceMeters : <flywheelRPS, rollerRPS, floorRPS>
    public static final InterpolatingMatrixTreeMap<Double, N3, N1> hubPresets =
        new InterpolatingMatrixTreeMap<>();
    public static final InterpolatingMatrixTreeMap<Double, N3, N1> ferryPresets =
        new InterpolatingMatrixTreeMap<>();

    public static final Distance hubMinDistance = Meters.of(1.89);
    public static final Distance hubMaxDistance = Meters.of(5.252);

    public static final Distance ferryMinDistance = Meters.of(0); // PLACEHOLDER
    public static final Distance ferryMaxDistance = Meters.of(20); // PLACEHOLDER

    public static final LinearVelocity hubProjectileHorizontalVelocity = MetersPerSecond.of(2.722);
    public static final LinearVelocity ferryProjectileHorizontalVelocity =
        MetersPerSecond.of(2.722); // PLACEHOLDER

    // distanceMeters : TOFSecs
    public static final InterpolatingDoubleTreeMap hubTOFs = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap ferryTOFs = new InterpolatingDoubleTreeMap();

    static {
      // hub presets
      hubPresets.put(2.0, vec3(44, 50, 50));
      hubPresets.put(2.97, vec3(50, 50, 50));
      hubPresets.put(4.26, vec3(57, 50, 50));
      hubPresets.put(5.012, vec3(61, 50, 50));

      // ferry presets
      ferryPresets.put(8.2705, vec3(61, 50, 50)); // PLACEHOLDER
      ferryPresets.put(16.541, vec3(90, 50, 50)); // PLACEHOLDER

      // hub TOFS
      hubTOFs.put(2.0, 0.885);
      hubTOFs.put(2.97, 1.05);
      hubTOFs.put(4.26, 1.275);
      hubTOFs.put(5.012, 1.41);

      // ferry TOFS
      ferryTOFs.put(8.2705, 2.0); // PLACEHOLDER
      ferryTOFs.put(16.541, 4.0); // PLACEHOLDER
    }
  }

  public static class ShooterConstants {
    public static final int flywheelMotorID = determineID(1); // 1
    public static final int flywheelFollowerMotorID = determineID(2); // 2

    public static final Voltage flywheelkS = Volts.of(0.23);
    public static final Per<VoltageUnit, AngularVelocityUnit> flywheelkV =
        Volts.per(RotationsPerSecond).ofNative(0.11);
    public static final Per<VoltageUnit, AngularVelocityUnit> flywheelkP =
        Volts.per(RotationsPerSecond).ofNative(0.3);

    public static final Current flywheelSupplyLimit = Amps.of(50);

    public static final AngularVelocity bangBangVelocityTolerance = RotationsPerSecond.of(3);

    public static final AngularVelocity windupVelocityTolerance = RotationsPerSecond.of(1);
    public static final AngularVelocity shootingVelocityTolerance = RotationsPerSecond.of(6);

    public static final double flywheelGearRatio = 1;
  }

  public static class HopperConstants {
    public static final int rollerMotorID = determineID(3);
    public static final int floorMotorID = determineID(4);

    public static final Voltage rollerkS = Volts.of(0.2);
    public static final Per<VoltageUnit, AngularVelocityUnit> rollerkV =
        Volts.per(RotationsPerSecond).ofNative(0.112);
    public static final Per<VoltageUnit, AngularVelocityUnit> rollerkP =
        Volts.per(RotationsPerSecond).ofNative(0.4);

    public static final Current rollerSupplyLimit = Amps.of(45);

    public static final Voltage floorkS = Volts.of(0.3);
    public static final Per<VoltageUnit, AngularVelocityUnit> floorkV =
        Volts.per(RotationsPerSecond).ofNative(0.23);
    public static final Per<VoltageUnit, AngularVelocityUnit> floorkP =
        Volts.per(RotationsPerSecond).ofNative(0.6);

    public static final Current floorSupplyLimit = Amps.of(45);

    public static final double rollerGearRatio = 1;
    public static final double floorGearRatio = 2;
  }

  public static class IntakeConstants {
    public static final int pivotMotorID = determineID(5);
    public static final int feedMotorID = determineID(6);

    public static final Voltage feedkS = Volts.of(0.3);

    public static final Per<VoltageUnit, AngularVelocityUnit> feedkV =
        Volts.per(RotationsPerSecond).ofNative(0.0928117314);

    public static final Per<VoltageUnit, AngularVelocityUnit> feedkP =
        Volts.per(RotationsPerSecond).ofNative(0.35);

    public static final Current feedSupplyLimit = Amps.of(40);

    public static final Voltage pivotkG = Volts.of(0.12365);
    public static final Voltage pivotkS = Volts.of(0.21257);

    public static final Per<VoltageUnit, AngularVelocityUnit> pivotkV =
        Volts.per(RotationsPerSecond).ofNative(8.7388);
    public static final Per<VoltageUnit, AngularAccelerationUnit> pivotkA =
        Volts.per(RotationsPerSecondPerSecond).ofNative(0.50935);

    public static final Per<VoltageUnit, AngleUnit> pivotkP = Volts.per(Rotations).ofNative(8);

    public static final AngularVelocity pivotVelocity = RotationsPerSecond.of(0.5);
    public static final AngularAcceleration pivotAcceleration = RotationsPerSecondPerSecond.of(0.5);

    public static final AngularVelocity shootingPivotVelocity = RotationsPerSecond.of(0.2);
    public static final AngularAcceleration shootingPivotAcceleration =
        RotationsPerSecondPerSecond.of(0.2);

    public static final double feedGearRatio = 1;
    public static final double pivotGearRatio = 72;

    public static final Angle pivotRaised = Rotations.of(0.22);
    public static final Angle pivotRaisedShooting = Rotations.of(0.28);

    public static final Angle pivotLowered = Rotations.of(0.54);
    public static final Angle pivotLoweredDepot = Rotations.of(0.45);

    public static final Angle pivotReverseSoftLimitThreshold = Rotations.of(0.17);
    public static final Angle pivotForwardSoftLimitThreshold = Rotations.of(0.59);

    public static final AngularVelocity feedSpeed = RotationsPerSecond.of(90);
  }

  public static class SwerveConstants {
    public static final Frequency odometryFrequency = Hertz.of(250);

    public static final Mass mass = Pounds.of(136);
    public static final MomentOfInertia moi = KilogramSquareMeters.of(8.79);

    public static final Current frontSlipCurrent = Amps.of(40);

    public static final LinearVelocity driverTranslationalVelocity = MetersPerSecond.of(4);
    public static final LinearVelocity driverTranslationalVelocityBump = MetersPerSecond.of(1.5);
    public static final AngularVelocity driverAngularVelocity = RadiansPerSecond.of(Math.PI * 2);

    public static final LinearVelocity driverTranslationalShootingVelocity =
        MetersPerSecond.of(1.5);

    public static final LinearVelocity profileTranslationalVelocity = MetersPerSecond.of(1);
    public static final LinearAcceleration profileTranslationalAcceleration =
        MetersPerSecondPerSecond.of(2);

    public static final AngularVelocity profileAngularVelocity = RadiansPerSecond.of(Math.PI * 2);
    public static final AngularAcceleration profileAngularAcceleration =
        RadiansPerSecondPerSecond.of(4);

    public static final Per<LinearVelocityUnit, DistanceUnit> poseTranslationalkP =
        MetersPerSecond.per(Meter).ofNative(3);
    public static final Per<LinearVelocityUnit, LinearVelocityUnit> poseTranslationalkD =
        MetersPerSecond.per(MetersPerSecond).ofNative(0.0);

    public static final Per<AngularVelocityUnit, AngleUnit> poseRotationkP =
        RadiansPerSecond.per(Radian).ofNative(5.5);
    public static final Per<AngularVelocityUnit, AngularVelocityUnit> poseRotationkD =
        RadiansPerSecond.per(RadiansPerSecond).ofNative(0.3);

    public static final boolean ignorePoseTolerance = true;

    public static final Translation2d poseTranslationTolerance = new Translation2d(0.03, 0.03);
    public static final Rotation2d poseRotationTolerance = Rotation2d.fromDegrees(1);

    public static final LinearVelocity translationalDeadband = MetersPerSecond.of(0.01);
    public static final AngularVelocity rotationalDeadband = RadiansPerSecond.of(0.01);
  }
}
