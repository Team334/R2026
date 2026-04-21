package frc.lib.math;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;

/**
 * Automatically resets a supplied LinearFilter after a given timeout has elapsed since the last
 * call to the filter.
 */
public class TimeoutLinearFilter {
  private final LinearFilter _filter;

  private final Timer _resetTimer = new Timer();
  private final double _timeout;

  /**
   * @param filter The filter.
   * @param timeout Timeout in seconds.
   */
  public TimeoutLinearFilter(LinearFilter filter, double timeout) {
    _filter = filter;
    _timeout = timeout;
  }

  /** Get the internal linear filter. */
  public LinearFilter getFilter() {
    return _filter;
  }

  /** Get the timeout in seconds. */
  public double getTimeout() {
    return _timeout;
  }

  /**
   * Calls the internal filter's {@link LinearFilter#calculate(double)}. If the specified timeout
   * has elapsed since the last calculate call, the internal filter is reset.
   */
  public double calculate(double input) {
    if (_resetTimer.hasElapsed(_timeout)) {
      _filter.reset();
    }

    double output = _filter.calculate(input);

    _resetTimer.restart();

    return output;
  }
}
