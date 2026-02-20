package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;

@Logged
public class ShotParameters {
  @Logged(name = "Flywheel Speed")
  private final MutAngularVelocity _flywheelSpeed = RotationsPerSecond.mutable(0);

  @Logged(name = "Hood Angle")
  private final MutAngle _hoodAngle = Rotations.mutable(0);

  @Logged(name = "Roller Speed")
  private final MutAngularVelocity _rollerSpeed = RotationsPerSecond.mutable(0);

  @Logged(name = "Floor Speed")
  private final MutAngularVelocity _floorSpeed = RotationsPerSecond.mutable(0);

  @Logged(name = "Shot Heading")
  private Rotation2d _shotHeading = new Rotation2d();

  @Logged(name = "Virtual Target")
  private Translation2d _virtualTarget = Translation2d.kZero;

  /**
   * If the robot-relative projectile velocity and the robot velocity vectors are strongly coupled,
   * the field-relative projectile velocity will have a larger error, given errors in the
   * robot-relative projectile velocity and robot velocity vectors.
   */
  @Logged(name = "Is Error Sensitive")
  public boolean isErrorSensitive = false;

  /**
   * The angle in degrees between the robot velocity vector and the vector pointing from the robot
   * to the target.
   */
  @Logged(name = "Coupling Degrees")
  public double couplingDegrees = 0;

  @Logged(name = "Newton Iterations")
  public int newtonIterations = 0;

  public ShotParameters() {}

  public AngularVelocity getFlywheelSpeed() {
    return _flywheelSpeed;
  }

  public Angle getHoodAngle() {
    return _hoodAngle;
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

  public void setPreset(Matrix<N4, N1> preset) {
    _flywheelSpeed.mut_setMagnitude(preset.get(0, 0));
    _hoodAngle.mut_setMagnitude(preset.get(1, 0));
    _rollerSpeed.mut_setMagnitude(preset.get(2, 0));
    _floorSpeed.mut_setMagnitude(preset.get(3, 0));
  }

  public void setShotHeading(Rotation2d shotHeading) {
    _shotHeading = shotHeading;
  }

  public void setVirtualTarget(Translation2d virtualTarget) {
    _virtualTarget = virtualTarget;
  }
}
