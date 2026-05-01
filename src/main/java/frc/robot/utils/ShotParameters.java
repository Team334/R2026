package frc.robot.utils;

import static edu.wpi.first.math.Nat.*;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;

public class ShotParameters {
  @Logged(name = "Flywheel Speed")
  private final MutAngularVelocity _flywheelSpeed = RotationsPerSecond.mutable(0);

  @Logged(name = "Roller Speed")
  private final MutAngularVelocity _rollerSpeed = RotationsPerSecond.mutable(0);

  @Logged(name = "Floor Speed")
  private final MutAngularVelocity _floorSpeed = RotationsPerSecond.mutable(0);

  @Logged(name = "Shot Heading")
  private double _shotHeading = 0;

  @Logged(name = "Target")
  private double[] _target = new double[3];

  @Logged(name = "Virtual Target")
  private double[] _virtualTarget = new double[3];

  @Logged(name = "In Bounds")
  public boolean inBounds = false;

  /**
   * If the robot-relative projectile velocity and the robot velocity vectors are strongly coupled,
   * the field-relative projectile velocity will have a larger error, given errors in the
   * robot-relative projectile velocity and robot velocity vectors.
   */
  @Logged(name = "Is Error Sensitive")
  public boolean isErrorSensitive = false;

  private final BooleanSupplier _isReadyToShoot;

  @Logged(name = "Is Manual")
  public boolean isManual = false;

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

  /**
   * Creates a new ShotParameters.
   *
   * @param shooterInTolerance Shooter flywheel is in tolerance of the shot parameters.
   * @param intakePivotInSafeZone The intake pivot is in its safe zone.
   * @param headingInTolerance Heading is in tolerance of the shoot parameters.
   * @param isShotValid Shot is valid depending on the hub status.
   */
  public ShotParameters(
      BooleanSupplier shooterInTolerance,
      BooleanSupplier intakePivotInSafeZone,
      BooleanSupplier headingInTolerance,
      BooleanSupplier isShotValid) {
    new Trigger(() -> isManual)
        .onTrue(
            runOnce(
                () -> {
                  // switch to manually set shot parameters
                  _shotHeading = 0;

                  _target = new double[3];
                  _virtualTarget = new double[3];

                  inBounds = false;

                  isErrorSensitive = false;
                  couplingDegrees = 0;

                  newtonIterations = 0;
                  failedToConverge = false;
                }));

    _isReadyToShoot =
        () -> {
          if (isManual) {
            // if shooting manually just wait for the shooter
            return shooterInTolerance.getAsBoolean();
          }

          return shooterInTolerance.getAsBoolean()
              && intakePivotInSafeZone.getAsBoolean()
              && headingInTolerance.getAsBoolean()
              && isShotValid.getAsBoolean()
              && inBounds
              && !failedToConverge;
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

  public double getShotHeading() {
    return _shotHeading;
  }

  public double getDistanceToVirtualTarget(Pose2d robotPose) {
    return Math.sqrt(
        Math.pow(_virtualTarget[0] - robotPose.getTranslation().getX(), 2)
            + Math.pow(_virtualTarget[1] - robotPose.getTranslation().getY(), 2));
  }

  /**
   * Checks that shot parameters are within tolerances, and {@link FieldUtil#isShotValid(Pose2d)}.
   */
  @Logged(name = "Is Ready To Shoot")
  public boolean isReadyToShoot() {
    return _isReadyToShoot.getAsBoolean();
  }

  public void setPreset(
      AngularVelocity flywheelSpeed, AngularVelocity rollerSpeed, AngularVelocity floorSpeed) {
    _flywheelSpeed.mut_setMagnitude(flywheelSpeed.in(RotationsPerSecond));
    _rollerSpeed.mut_setMagnitude(rollerSpeed.in(RotationsPerSecond));
    _floorSpeed.mut_setMagnitude(floorSpeed.in(RotationsPerSecond));
  }

  public void setPreset(Matrix<N3, N1> preset) {
    _flywheelSpeed.mut_setMagnitude(preset.get(0, 0));
    _rollerSpeed.mut_setMagnitude(preset.get(1, 0));
    _floorSpeed.mut_setMagnitude(preset.get(2, 0));
  }

  public void setShotHeading(double shotHeading) {
    _shotHeading = shotHeading;
  }

  public void setTarget(double targetX, double targetY) {
    _target[0] = targetX;
    _target[1] = targetY;
  }

  public void setVirtualTarget(double virtualTargetX, double virtualTargetY) {
    _virtualTarget[0] = virtualTargetX;
    _virtualTarget[1] = virtualTargetY;
  }
}
