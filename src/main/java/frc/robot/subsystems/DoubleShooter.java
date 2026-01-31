// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

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
import frc.robot.Constants.DoubleShooterConstants;


public class DoubleShooter extends AdvancedSubsystem {
private final TalonFX _frontMotor =
      new TalonFX(DoubleShooterConstants.frontMotorID, Constants.subsystemBus);
  private final TalonFX _backMotor =
      new TalonFX(DoubleShooterConstants.backMotorID, Constants.subsystemBus);

  private final VelocityVoltage _velocitySetter = new VelocityVoltage(0);
  private final DutyCycleOut _dutyCycleSetter = new DutyCycleOut(0);

  private final StatusSignal<AngularVelocity> _frontVelocityGetter = _frontMotor.getVelocity();
  private final StatusSignal<AngularVelocity> _backVelocityGetter = _backMotor.getVelocity();

  @Logged(name = "Desired Front Speed")
  private final MutAngularVelocity _desiredFrontSpeed = RotationsPerSecond.mutable(0);

  @Logged(name = "Desired Back Speed")
  private final MutAngularVelocity _desiredBackSpeed = RotationsPerSecond.mutable(0);

  @Logged(name = "Velocity Threshold")
  private final AngularVelocity velocityThreshold = RotationsPerSecond.of(3);

  public DoubleShooter() {
    var frontMotorConfig = new TalonFXConfiguration();
    var backMotorConfig = new TalonFXConfiguration();

    // front motor configs
    frontMotorConfig.CurrentLimits.StatorCurrentLimit = 100;
    frontMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    frontMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = false;

    frontMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    frontMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    frontMotorConfig.Slot0.kS = DoubleShooterConstants.frontFlywheelkS.in(Volts);
    frontMotorConfig.Slot0.kV = DoubleShooterConstants.frontFlywheelkV.in(Volts.per(RotationsPerSecond));

    frontMotorConfig.Slot0.kP = DoubleShooterConstants.frontFlywheelkP.in(Volts.per(RotationsPerSecond));

    frontMotorConfig.Feedback.SensorToMechanismRatio = DoubleShooterConstants.frontFlywheelGearRatio;

    // back motor configs
    backMotorConfig.CurrentLimits.StatorCurrentLimit = 100;
    backMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    backMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = false;

    backMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    backMotorConfig.Slot0.kS = DoubleShooterConstants.backFlywheelkS.in(Volts);
    backMotorConfig.Slot0.kV = DoubleShooterConstants.backFlywheelkV.in(Volts.per(RotationsPerSecond));

    backMotorConfig.Slot0.kP = DoubleShooterConstants.backFlywheelkP.in(Volts.per(RotationsPerSecond));

    backMotorConfig.Feedback.SensorToMechanismRatio = DoubleShooterConstants.backFlywheelGearRatio;

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

    if (Math.abs(errorRps) > velocityThreshold.in(RotationsPerSecond)) {
      _frontMotor.setControl(_dutyCycleSetter.withOutput(Math.signum(errorRps)));
    } else {
      _frontMotor.setControl(_velocitySetter.withVelocity(desiredFrontSpeed));
    }
  }

  private void setBackSpeed(AngularVelocity desiredBackSpeed) {
    double errorRps = desiredBackSpeed.minus(getBackSpeed()).in(RotationsPerSecond);

    if (Math.abs(errorRps) > velocityThreshold.in(RotationsPerSecond)) {
      _backMotor.setControl(_dutyCycleSetter.withOutput(Math.signum(errorRps)));
    } else {
      _backMotor.setControl(_velocitySetter.withVelocity(desiredBackSpeed));
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
