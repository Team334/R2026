package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;

@Logged
public class ShotParameters {
  @Logged(name = "Flywheel Speed")
  private final MutAngularVelocity _flywheelSpeed = RotationsPerSecond.mutable(0);

  @Logged(name = "Hood Angle")
  private final MutAngle _hoodAngle = Rotations.mutable(0);

  @Logged(name = "Shot Heading")
  private final Rotation2d _shotHeading = new Rotation2d();

  @Logged(name = "Virtual Target")
  private final Translation2d _virtualTarget = Translation2d.kZero;

  @Logged(name = "Is Valid")
  public boolean isValid = true;

  public ShotParameters() {}

  public MutAngularVelocity getFlywheelSpeed() {
    return _flywheelSpeed;
  }

  public MutAngle getHoodAngle() {
    return _hoodAngle;
  }

  public Rotation2d getShotHeading() {
    return _shotHeading;
  }

  // <flywheelRPS, hoodAngleRot>
  public void set(Matrix<N2, N1> vec) {
    _flywheelSpeed.mut_setMagnitude(vec.get(0, 0));
    _hoodAngle.mut_setMagnitude(vec.get(1, 0));
  }
}
