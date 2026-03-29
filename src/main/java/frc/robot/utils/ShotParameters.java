package frc.robot.utils;

import static edu.wpi.first.math.Nat.*;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

@Logged
public class ShotParameters {
  @Logged(name = "Flywheel Speed")
  private final MutAngularVelocity _flywheelSpeed = RotationsPerSecond.mutable(0);

  @Logged(name = "Roller Speed")
  private final MutAngularVelocity _rollerSpeed = RotationsPerSecond.mutable(0);

  @Logged(name = "Floor Speed")
  private final MutAngularVelocity _floorSpeed = RotationsPerSecond.mutable(0);

  @Logged(name = "Shot Heading")
  private Rotation2d _shotHeading = new Rotation2d();

  @Logged(name = "Target")
  private Pose2d _target = Pose2d.kZero;

  @Logged(name = "Virtual Target")
  private Pose2d _virtualTarget = Pose2d.kZero;

  @Logged(name = "In Bounds")
  public boolean inBounds = false;

  /**
   * If the robot-relative projectile velocity and the robot velocity vectors are strongly coupled,
   * the field-relative projectile velocity will have a larger error, given errors in the
   * robot-relative projectile velocity and robot velocity vectors.
   */
  @Logged(name = "Is Error Sensitive")
  public boolean isErrorSensitive = false;

  @NotLogged private final BooleanSupplier _isReadyToShoot;

  /**
   * The angle in degrees between the robot velocity vector and the vector pointing from the robot
   * to the target.
   */
  @Logged(name = "Coupling Degrees")
  public double couplingDegrees = 0;

  @Logged(name = "Newton Iterations")
  public int newtonIterations = 0;

  @Logged(name = "Failed To Converge")
  public boolean failedToConverge = false;

  public static Matrix<N3, N1> vec3(double a, double b, double c) {
    return new Matrix<N3, N1>(N3(), N1(), new double[] {a, b, c});
  }

  public ShotParameters() {
    _isReadyToShoot = () -> true;
  }

  public ShotParameters(
      BooleanSupplier shooterInTolerance,
      Supplier<Pose2d> robotPoseSupplier,
      Rotation2d headingTolerance) {
    _isReadyToShoot =
        () -> {
          return shooterInTolerance.getAsBoolean()
              // && Math.abs(_shotHeading.minus(robotPoseSupplier.get().getRotation()).getDegrees())
              //     < headingTolerance.getDegrees()
              && FieldUtil.isShotValid(robotPoseSupplier.get());
          // && inBounds
          // && !failedToConverge;
        };
  }

  public AngularVelocity getFlywheelSpeed() {
    return _flywheelSpeed;
  }

  public AngularVelocity getRollerSpeed() {
    return _rollerSpeed;
  }

  public AngularVelocity getFloorSpeed() {
    return _floorSpeed;
  }

  public Rotation2d getShotHeading() {
    return _shotHeading;
  }

  public Pose2d getTarget() {
    return _target;
  }

  public Pose2d getVirtualTarget() {
    return _virtualTarget;
  }

  /**
   * Checks that shot parameters are within tolerances, and {@link FieldUtil#isShotValid(Pose2d)}.
   */
  @Logged(name = "Is Ready To Shoot")
  public boolean isReadyToShoot() {
    return _isReadyToShoot.getAsBoolean();
  }

  public void setPreset(Matrix<N3, N1> preset) {
    _flywheelSpeed.mut_setMagnitude(preset.get(0, 0));
    _rollerSpeed.mut_setMagnitude(preset.get(1, 0));
    _floorSpeed.mut_setMagnitude(preset.get(2, 0));
  }

  public void setShotHeading(Rotation2d shotHeading) {
    _shotHeading = shotHeading;
  }

  public void setTarget(Translation2d target) {
    _target = new Pose2d(target, Rotation2d.kZero);
  }

  public void setVirtualTarget(Translation2d virtualTarget) {
    _virtualTarget = new Pose2d(virtualTarget, Rotation2d.kZero);
  }
}
