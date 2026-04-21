package frc.lib.math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;

public final class FilteredPIDController {
  private final double kP;
  private final double kD;

  private final double period;

  private boolean _filterSetpointVelocity;
  private final LinearFilter _setpointVelocityFilter;

  private final Timer _setpointVelocityFilterTimer = new Timer();

  private double _prevSetpoint;
  private double _prevMeasurement;

  private boolean _continuous;
  private double _maximumInput;
  private double _minimumInput;

  public FilteredPIDController(double kP, double kD, double period, double tau) {
    this.kP = kP;
    this.kD = kD;

    this.period = period;

    _setpointVelocityFilter = LinearFilter.singlePoleIIR(tau, this.period);
  }

  public boolean isFilteringSetpointVelocity() {
    return _filterSetpointVelocity;
  }

  public void enableSetpointVelocityFilter() {
    _filterSetpointVelocity = true;
  }

  public void disableSetpointVelocityFilter() {
    _filterSetpointVelocity = false;
  }

  public boolean isContinuousInput() {
    return _continuous;
  }

  public void enableContinuousInput(double minimumInput, double maximumInput) {
    _continuous = true;
    _minimumInput = minimumInput;
    _maximumInput = maximumInput;
  }

  public void disableContinuousInput() {
    _continuous = false;
  }

  public double calculate(double measurement, double setpoint) {
    double error;

    double measurementDerivative;
    double setpointDerivative;

    if (_continuous) {
      double bound = (_maximumInput - _minimumInput) / 2.0;

      error = MathUtil.inputModulus(setpoint - measurement, -bound, bound);

      measurementDerivative =
          MathUtil.inputModulus(measurement - _prevMeasurement, -bound, bound) / period;
      setpointDerivative = MathUtil.inputModulus(setpoint - _prevSetpoint, -bound, bound) / period;
    } else {
      error = setpoint - measurement;

      measurementDerivative = measurement - _prevMeasurement;
      setpointDerivative = setpoint - _prevSetpoint;
    }

    if (_filterSetpointVelocity) {
      // TODO make reset time a constant?
      if (_setpointVelocityFilterTimer.hasElapsed(2)) {
        _setpointVelocityFilter.reset();
      }

      setpointDerivative = _setpointVelocityFilter.calculate(setpointDerivative);

      _setpointVelocityFilterTimer.restart();
    }

    _prevMeasurement = measurement;
    _prevSetpoint = setpoint;

    return kP * error + kD * (setpointDerivative - measurementDerivative);
  }
}
