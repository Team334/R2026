package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;

public class ShotPreset {
  private final MutAngularVelocity _flywheelSpeed = RotationsPerSecond.mutable(0);
  private final MutAngle _hoodAngle = Rotations.mutable(0);

  private boolean _isValid = true;

  public ShotPreset() {}

  public ShotPreset(AngularVelocity flywheelSpeed, Angle hoodAngle) {
    _flywheelSpeed.mut_setMagnitude(flywheelSpeed.in(RotationsPerSecond));
    _hoodAngle.mut_setMagnitude(hoodAngle.in(Rotations));
  }

  public MutAngularVelocity getFlywheelSpeed() {
    return _flywheelSpeed;
  }

  public MutAngle getHoodAngle() {
    return _hoodAngle;
  }

  public boolean is_isValid() {
    return _isValid;
  }

  // <flywheelRPS, hoodAngleRot>
  public ShotPreset set(Matrix<N2, N1> preset) {
    _flywheelSpeed.mut_setMagnitude(preset.get(0, 0));
    _hoodAngle.mut_setMagnitude(preset.get(1, 0));

    return this;
  }
}
