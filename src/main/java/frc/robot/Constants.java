// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.math.Nat.*;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
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
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Frequency simNotifierFrequency = Hertz.of(200);

  public static final CANBus subsystemBus = new CANBus("canivore");

  public static final Time shotTimeScaler = Seconds.of(0.2);

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

    public static final Translation2d blueFerryBottom = new Translation2d();
    public static final Translation2d blueFerryTop = new Translation2d();

    public static final Translation2d redFerryBottom = new Translation2d();
    public static final Translation2d redFerryTop = new Translation2d();

    public static final double ferryXThresholdBlue = 3;
    public static final double ferryXThresholdRed = 7;

    public static final double ferryYThreshold = 6;

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
    public static final double zBoundMargin = 0.01;
  }

  public static class ShooterConstants {
    public static final int flywheelMotorID = 0;
    public static final int flywheelFollowerMotorID = 41;
    public static final int hoodMotorID = 40;

    public static final Voltage flywheelkS = Volts.of(0.39);
    public static final Per<VoltageUnit, AngularVelocityUnit> flywheelkV =
        Volts.per(RotationsPerSecond).ofNative(0.27);
    public static final Per<VoltageUnit, AngularVelocityUnit> flywheelkP =
        Volts.per(RotationsPerSecond).ofNative(1.3);

    public static final Voltage hoodkS = Volts.of(0);
    public static final Voltage hoodkG = Volts.of(0);
    public static final Per<VoltageUnit, AngularVelocityUnit> hoodkV =
        Volts.per(RotationsPerSecond).ofNative(0);
    public static final Per<VoltageUnit, AngularAccelerationUnit> hoodkA =
        Volts.per(RotationsPerSecondPerSecond).ofNative(0);
    public static final Per<VoltageUnit, AngleUnit> hoodkP = Volts.per(Rotations).ofNative(0);

    public static final AngularVelocity hoodVelocity = RotationsPerSecond.of(2);
    public static final AngularAcceleration hoodAcceleration = RotationsPerSecondPerSecond.of(5);

    public static final Angle hoodForwardSoftLimitThreshold = Rotations.of(1);
    public static final Angle hoodReverseSoftLimitThreshold = Rotations.of(0);

    public static final double flywheelGearRatio = 3;
    public static final double hoodGearRatio = 3;

    private static Matrix<N2, N1> vec2d(double a, double b) {
      return new Matrix<N2, N1>(N2(), N1(), new double[] {a, b});
    }

    // state = <speedRps, angleRot>
    public static Matrix<N2, N1> spitState = vec2d(0, 0);

    // distanceMeters : state
    public static InterpolatingMatrixTreeMap<Double, N2, N1> scoreTable =
        new InterpolatingMatrixTreeMap<>();
    public static InterpolatingMatrixTreeMap<Double, N2, N1> ferryTable =
        new InterpolatingMatrixTreeMap<>();

    static {
      // score table
      scoreTable.put(1.0, vec2d(1.0, 1.0));

      // ferry table
      ferryTable.put(1.0, vec2d(1.0, 1.0));
    }
  }

  public static class HopperConstants {
    public static final int rollerMotorID = 20;
    public static final int floorMotorID = 21;

    public static final AngularVelocity floorShootSpeed = RotationsPerSecond.of(0);
    public static final AngularVelocity rollerShootSpeed = RotationsPerSecond.of(0);

    public static final Voltage rollerkS = Volts.of(0.39);
    public static final Per<VoltageUnit, AngularVelocityUnit> rollerkV =
        Volts.per(RotationsPerSecond).ofNative(0.27);
    public static final Per<VoltageUnit, AngularVelocityUnit> rollerkP =
        Volts.per(RotationsPerSecond).ofNative(1.3);

    public static final Voltage floorkS = Volts.of(0.47);
    public static final Per<VoltageUnit, AngularVelocityUnit> floorkV =
        Volts.per(RotationsPerSecond).ofNative(0.3);
    public static final Per<VoltageUnit, AngularVelocityUnit> floorkP =
        Volts.per(RotationsPerSecond).ofNative(1);

    public static final double rollerGearRatio = 3;
    public static final double floorGearRatio = 3;
  }

  public static class IntakeConstants {
    public static final int feedMotorID = 10;
    public static final int pivotMotorID = 11;

    public static final Voltage feedkS = Volts.of(0.3);

    public static final Per<VoltageUnit, AngularVelocityUnit> feedkV =
        Volts.per(RotationsPerSecond).ofNative(0.15);

    public static final Per<VoltageUnit, AngularVelocityUnit> feedkP =
        Volts.per(RotationsPerSecond).ofNative(0.6);

    public static final Voltage pivotkG = Volts.of(0.2);
    public static final Voltage pivotkS = Volts.of(0.1);

    public static final Per<VoltageUnit, AngularVelocityUnit> pivotkV =
        Volts.per(RotationsPerSecond).ofNative(1);
    public static final Per<VoltageUnit, AngularAccelerationUnit> pivotkA =
        Volts.per(RotationsPerSecondPerSecond).ofNative(0.1);

    public static final Per<VoltageUnit, AngleUnit> pivotkP = Volts.per(Rotations).ofNative(0.8);

    public static final AngularVelocity pivotVelocity = RotationsPerSecond.of(2);
    public static final AngularAcceleration pivotAcceleration = RotationsPerSecondPerSecond.of(5);

    public static final double feedGearRatio = 1.5;
    public static final double pivotGearRatio = 20;

    public static final Angle pivotRaised = Rotations.of(0.25);
    public static final Angle pivotTucked = Rotations.of(0.5);
    public static final Angle pivotLowered = Rotations.of(0.6);

    public static final Angle pivotForwardSoftLimitThreshold = Rotations.of(0.7);
    public static final Angle pivotReverseSoftLimitThreshold = Rotations.of(0.2);

    public static final AngularVelocity feedSpeed = RotationsPerSecond.of(30);
  }

  public static class ClimbConstants {
    // Change ALL values here after testing
    public static final int climbMotorID = 20;

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

    public static final Angle forwardSoftLimitThreshold = Rotations.of(11);
    public static final Angle reverseSoftLimitThreshold = Rotations.of(-1);

    public static final double climbGearRatio = 3;
  }

  public static class SwerveConstants {
    public static final Frequency odometryFrequency = Hertz.of(250);

    public static final Mass mass = Pounds.of(136.38);
    public static final MomentOfInertia moi = KilogramSquareMeters.of(0);

    public static final LinearVelocity driverTranslationalVelocity = MetersPerSecond.of(4);
    public static final AngularVelocity driverAngularVelocity = RadiansPerSecond.of(Math.PI);

    public static final LinearVelocity driverTranslationalShootingVelocity = MetersPerSecond.of(2);

    public static final LinearVelocity profileTranslationalVelocity = MetersPerSecond.of(1);
    public static final LinearAcceleration profileTranslationalAcceleration =
        MetersPerSecondPerSecond.of(2);

    public static final AngularVelocity profileAngularVelocity = RadiansPerSecond.of(Math.PI);
    public static final AngularAcceleration profileAngularAcceleration =
        RadiansPerSecondPerSecond.of(Math.PI * 2);

    public static final Per<LinearVelocityUnit, DistanceUnit> poseTranslationalkP =
        MetersPerSecond.per(Meter).ofNative(0);
    public static final Per<AngularVelocityUnit, AngleUnit> poseRotationkP =
        RadiansPerSecond.per(Radian).ofNative(0);

    public static final boolean ignorePoseTolerance = true;

    public static final Translation2d poseTranslationTolerance = new Translation2d(0.03, 0.03);
    public static final Rotation2d poseRotationTolerance = Rotation2d.fromDegrees(1);

    public static LinearVelocity translationalDeadband = MetersPerSecond.of(0.01);
    public static AngularVelocity rotationalDeadband = RadiansPerSecond.of(0.01);
  }
}
