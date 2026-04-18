package frc.lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;

public final class FilteredPIDController {
  private final double kP;
  private final double kD;

  private final double period;

  private boolean _filterSetpointVelocity;
  private LinearFilter _setpointVelocityFilter;

  private double _prevSetpoint;
  private double _prevMeasurement;

  private boolean _continuous;
  private double _maximumInput;
  private double _minimumInput;

  public FilteredPIDController(double kP, double kD, double period) {
    this.kP = kP;
    this.kD = kD;

    this.period = period;
  }

  public boolean isFilteringSetpointVelocity() {
    return _filterSetpointVelocity;
  }

  public void enableSetpointVelocityFilter(double tau) {
    _filterSetpointVelocity = true;
    _setpointVelocityFilter = LinearFilter.singlePoleIIR(tau, period);
  }

  public void disableSetpointVelocityFilter() {
    _filterSetpointVelocity = false;
    _setpointVelocityFilter = null;
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

      measurementDerivative = MathUtil.inputModulus(measurement - _prevMeasurement, -bound, bound) / period;
      setpointDerivative = MathUtil.inputModulus(setpoint - _prevSetpoint, -bound, bound) / period;
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

    return kP * error + kD * (setpointDerivative - measurementDerivative);
  }
}
