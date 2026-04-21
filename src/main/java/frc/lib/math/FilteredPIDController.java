package frc.lib.math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;

public final class FilteredPIDController {
  private final double _kP;
  private final double _kD;

  private final double _period;

  private boolean _filterSetpointVelocity;
  private final TimeoutLinearFilter _setpointVelocityFilter;

  private double _prevSetpoint;
  private double _prevMeasurement;

  private boolean _continuous;
  private double _maximumInput;
  private double _minimumInput;

  public FilteredPIDController(double kP, double kD, double period, double tau) {
    this._kP = kP;
    this._kD = kD;

    this._period = period;

    _setpointVelocityFilter =
        new TimeoutLinearFilter(LinearFilter.singlePoleIIR(tau, this._period), 2);
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
          MathUtil.inputModulus(measurement - _prevMeasurement, -bound, bound) / _period;
      setpointDerivative = MathUtil.inputModulus(setpoint - _prevSetpoint, -bound, bound) / _period;
    } else {
      error = setpoint - measurement;

      measurementDerivative = measurement - _prevMeasurement;
      setpointDerivative = setpoint - _prevSetpoint;
    }

    if (_filterSetpointVelocity) {
      setpointDerivative = _setpointVelocityFilter.calculate(setpointDerivative);
    }

    _prevMeasurement = measurement;
    _prevSetpoint = setpoint;

    return _kP * error + _kD * (setpointDerivative - measurementDerivative);
  }
}
