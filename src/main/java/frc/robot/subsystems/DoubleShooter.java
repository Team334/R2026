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
  private final TalonFX _flywheelMotor =
      new TalonFX(DoubleShooterConstants.flywheelMotorID, Constants.subsystemBus);

  private final VelocityVoltage _velocitySetter = new VelocityVoltage(0);
  private final DutyCycleOut _dutyCycleSetter = new DutyCycleOut(0);

  private final StatusSignal<AngularVelocity> _flywheelVelocityGetter =
      _flywheelMotor.getVelocity();

  @Logged(name = "Desired Flywheel Speed")
  private final MutAngularVelocity _desiredFlywheelSpeed = RotationsPerSecond.mutable(0);

  @Logged(name = "Velocity Threshold")
  private final AngularVelocity velocityThreshold = RotationsPerSecond.of(3);

  public DoubleShooter() {
    var flywheelMotorConfig = new TalonFXConfiguration();

    // front motor configs
    flywheelMotorConfig.CurrentLimits.StatorCurrentLimit = 100;
    flywheelMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    flywheelMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = false;

    flywheelMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    flywheelMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    flywheelMotorConfig.Slot0.kS = DoubleShooterConstants.flywheelkS.in(Volts);
    flywheelMotorConfig.Slot0.kV =
        DoubleShooterConstants.flywheelkV.in(Volts.per(RotationsPerSecond));

    flywheelMotorConfig.Slot0.kP =
        DoubleShooterConstants.flywheelkP.in(Volts.per(RotationsPerSecond));

    flywheelMotorConfig.Feedback.SensorToMechanismRatio = DoubleShooterConstants.flywheelGearRatio;

    CTREUtil.attempt(
        () -> _flywheelMotor.getConfigurator().apply(flywheelMotorConfig), _flywheelMotor);

    CTREUtil.attempt(() -> _flywheelMotor.optimizeBusUtilization(), _flywheelMotor);

    CTREUtil.attempt(
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                _flywheelVelocityGetter,
                _flywheelMotor.getSupplyCurrent(),
                _flywheelMotor.getStatorCurrent()),
        _flywheelMotor);

    FaultLogger.register(_flywheelMotor);

    // EVERYTHING BELOW IS TEMPORARY
    DogLog.tunable(
        "Desired Flywheel Speed RPS",
        0.0,
        newRps -> _desiredFlywheelSpeed.mut_setMagnitude(newRps));

    final CoastOut coast = new CoastOut();

    setDefaultCommand(
        run(
            () -> {
              _flywheelMotor.setControl(coast);
            }));
  }

  private void setFlywheelSpeed(AngularVelocity desiredFrontSpeed) {
    double errorRps = desiredFrontSpeed.minus(getFrontSpeed()).in(RotationsPerSecond);

    if (Math.abs(errorRps) > velocityThreshold.in(RotationsPerSecond)) {
      _flywheelMotor.setControl(_dutyCycleSetter.withOutput(Math.signum(errorRps)));
    } else {
      _flywheelMotor.setControl(_velocitySetter.withVelocity(desiredFrontSpeed));
    }
  }

  /** Shoot. */
  public Command shoot() {
    return run(() -> {
          setFlywheelSpeed(_desiredFlywheelSpeed);
        })
        .withName("Shoot");
  }

  @Logged(name = "Front Speed")
  public AngularVelocity getFrontSpeed() {
    return _flywheelVelocityGetter.refresh().getValue();
  }

  @Override
  public void close() {
    _flywheelMotor.close();
  }
}
