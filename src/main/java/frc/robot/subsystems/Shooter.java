package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AdvancedSubsystem;
import frc.lib.CTREUtil;
import frc.lib.FaultLogger;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends AdvancedSubsystem {
  private final TalonFX _frontMotor =
      new TalonFX(ShooterConstants.frontMotorID, Constants.subsystemBus);
  private final TalonFX _backMotor =
      new TalonFX(ShooterConstants.backMotorID, Constants.subsystemBus);

  private final VelocityVoltage _velocitySetter = new VelocityVoltage(0);
  private final DutyCycleOut _dutyCycleSetter = new DutyCycleOut(0);

  private final StatusSignal<AngularVelocity> _frontVelocityGetter = _frontMotor.getVelocity();
  private final StatusSignal<AngularVelocity> _backVelocityGetter = _backMotor.getVelocity();

  @Logged(name = "Desired Front Speed")
  private final MutAngularVelocity _desiredFrontSpeed = RotationsPerSecond.mutable(0);

  @Logged(name = "Desired Back Speed")
  private final MutAngularVelocity _desiredBackSpeed = RotationsPerSecond.mutable(0);

  @Logged(name = "Front Is Duty Cycle")
  private boolean _frontIsDutyCycle = false;

  @Logged(name = "Back Is Duty Cycle")
  private boolean _backIsDutyCycle = false;

  private final AngularVelocity threshold = RotationsPerSecond.of(1);
  private final AngularVelocity smallerThreshold = RotationsPerSecond.of(0.08);

  public Shooter() {
    var frontMotorConfig = new TalonFXConfiguration();
    var backMotorConfig = new TalonFXConfiguration();

    // front motor configs
    frontMotorConfig.CurrentLimits.StatorCurrentLimit = 100;
    frontMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    frontMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = false;

    frontMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    frontMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    frontMotorConfig.Slot0.kS = ShooterConstants.frontFlywheelkS.in(Volts);
    frontMotorConfig.Slot0.kV = ShooterConstants.frontFlywheelkV.in(Volts.per(RotationsPerSecond));

    frontMotorConfig.Slot0.kP = ShooterConstants.frontFlywheelkP.in(Volts.per(RotationsPerSecond));

    frontMotorConfig.Feedback.SensorToMechanismRatio = ShooterConstants.frontFlywheelGearRatio;

    // back motor configs
    backMotorConfig.CurrentLimits.StatorCurrentLimit = 100;
    backMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    backMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = false;

    backMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    backMotorConfig.Slot0.kS = ShooterConstants.backFlywheelkS.in(Volts);
    backMotorConfig.Slot0.kV = ShooterConstants.backFlywheelkV.in(Volts.per(RotationsPerSecond));

    backMotorConfig.Slot0.kP = ShooterConstants.backFlywheelkP.in(Volts.per(RotationsPerSecond));

    backMotorConfig.Feedback.SensorToMechanismRatio = ShooterConstants.backFlywheelGearRatio;

    CTREUtil.attempt(() -> _frontMotor.getConfigurator().apply(frontMotorConfig), _frontMotor);
    CTREUtil.attempt(() -> _backMotor.getConfigurator().apply(backMotorConfig), _backMotor);

    CTREUtil.attempt(() -> _frontMotor.optimizeBusUtilization(), _frontMotor);
    CTREUtil.attempt(() -> _backMotor.optimizeBusUtilization(), _backMotor);

    CTREUtil.attempt(
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                _frontVelocityGetter,
                _frontMotor.getSupplyCurrent(),
                _frontMotor.getStatorCurrent()),
        _frontMotor);

    CTREUtil.attempt(
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                _backVelocityGetter,
                _backMotor.getSupplyCurrent(),
                _backMotor.getStatorCurrent()),
        _backMotor);

    FaultLogger.register(_frontMotor);
    FaultLogger.register(_backMotor);

    // EVERYTHING BELOW IS TEMPORARY
    DogLog.tunable(
        "Desired Front Speed RPS", 0.0, newRps -> _desiredFrontSpeed.mut_setMagnitude(newRps));
    DogLog.tunable(
        "Desired Back Speed RPS", 0.0, newRps -> _desiredBackSpeed.mut_setMagnitude(newRps));

    final CoastOut coast = new CoastOut();

    setDefaultCommand(
        run(
            () -> {
              _frontMotor.setControl(coast);
              _backMotor.setControl(coast);
            }));
  }

  private void setFrontSpeed(AngularVelocity desiredFrontSpeed) {
    double errorRps = desiredFrontSpeed.minus(getFrontSpeed()).in(RotationsPerSecond);

    if (!_frontIsDutyCycle && Math.abs(errorRps) > threshold.in(RotationsPerSecond)) {
      _frontMotor.setControl(_dutyCycleSetter.withOutput(Math.signum(errorRps)));
      _frontIsDutyCycle = true;
    }

    if (_frontIsDutyCycle && Math.abs(errorRps) < smallerThreshold.in(RotationsPerSecond)) {
      _frontMotor.setControl(_velocitySetter.withVelocity(desiredFrontSpeed));
      _frontIsDutyCycle = false;
    }
  }

  private void setBackSpeed(AngularVelocity desiredBackSpeed) {
    double errorRps = desiredBackSpeed.minus(getBackSpeed()).in(RotationsPerSecond);

    if (!_backIsDutyCycle && Math.abs(errorRps) > threshold.in(RotationsPerSecond)) {
      _backMotor.setControl(_dutyCycleSetter.withOutput(Math.signum(errorRps)));
      _backIsDutyCycle = true;
    }

    if (_backIsDutyCycle && Math.abs(errorRps) < smallerThreshold.in(RotationsPerSecond)) {
      _backMotor.setControl(_velocitySetter.withVelocity(desiredBackSpeed));
      _backIsDutyCycle = false;
    }
  }

  /** Shoot. */
  public Command shoot() {
    return run(() -> {
          setFrontSpeed(_desiredFrontSpeed);
          setBackSpeed(_desiredBackSpeed);
        })
        .withName("Shoot");
  }

  @Logged(name = "Front Speed")
  public AngularVelocity getFrontSpeed() {
    return _frontVelocityGetter.refresh().getValue();
  }

  @Logged(name = "Back Speed")
  public AngularVelocity getBackSpeed() {
    return _backVelocityGetter.refresh().getValue();
  }

  @Override
  public void close() {
    _frontMotor.close();
    _backMotor.close();
  }
}
