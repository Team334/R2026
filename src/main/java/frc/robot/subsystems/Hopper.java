// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.epilogue.Logged;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AdvancedSubsystem;
import frc.lib.CTREUtil;
import frc.lib.FaultLogger;
import frc.robot.Constants;
import frc.robot.Constants.HopperConstants;

public class Hopper extends AdvancedSubsystem {
  /** Creates a new Hopper. */

  private final TalonFX _floorMotor =
      new TalonFX(HopperConstants.floorMotorID, Constants.subsystemBus);
  private final TalonFX _feedMotor =
      new TalonFX(HopperConstants.feedMotorID, Constants.subsystemBus);

  private final VelocityVoltage _velocitySetter = new VelocityVoltage(0);
  private final DutyCycleOut _dutyCycleSetter = new DutyCycleOut(0);

  private final StatusSignal<AngularVelocity> _floorVelocityGetter = _floorMotor.getVelocity();
  private final StatusSignal<AngularVelocity> _feedVelocityGetter = _feedMotor.getVelocity();

  @Logged(name = "Desired Floor Speed")
  private final MutAngularVelocity _desiredFloorSpeed = RotationsPerSecond.mutable(0);

  @Logged(name = "Desired Feed Speed")
  private final MutAngularVelocity _desiredFeedSpeed = RotationsPerSecond.mutable(0);

  @Logged(name = "Velocity Threshold")
  private final AngularVelocity velocityThreshold = RotationsPerSecond.of(3);


  public Hopper() {
    var floorMotorConfig = new TalonFXConfiguration();
    var feedMotorConfig = new TalonFXConfiguration();

    // floor motor configs
    floorMotorConfig.CurrentLimits.StatorCurrentLimit = 100;
    floorMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    floorMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = false;

    floorMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    floorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    floorMotorConfig.Slot0.kS = HopperConstants.floorkS.in(Volts);
    floorMotorConfig.Slot0.kV = HopperConstants.floorkV.in(Volts.per(RotationsPerSecond));

    floorMotorConfig.Slot0.kP = HopperConstants.floorkP.in(Volts.per(RotationsPerSecond));

    floorMotorConfig.Feedback.SensorToMechanismRatio = HopperConstants.floorGearRatio;

    // feed motor configs
    feedMotorConfig.CurrentLimits.StatorCurrentLimit = 100;
    feedMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    feedMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = false;

    feedMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    feedMotorConfig.Slot0.kS = HopperConstants.feedkS.in(Volts);
    feedMotorConfig.Slot0.kV = HopperConstants.feedkV.in(Volts.per(RotationsPerSecond));

    feedMotorConfig.Slot0.kP = HopperConstants.feedkP.in(Volts.per(RotationsPerSecond));

    feedMotorConfig.Feedback.SensorToMechanismRatio = HopperConstants.feedGearRatio;

    CTREUtil.attempt(() -> _floorMotor.getConfigurator().apply(floorMotorConfig), _floorMotor);
    CTREUtil.attempt(() -> _feedMotor.getConfigurator().apply(feedMotorConfig), _feedMotor);

    CTREUtil.attempt(() -> _floorMotor.optimizeBusUtilization(), _floorMotor);
    CTREUtil.attempt(() -> _feedMotor.optimizeBusUtilization(), _feedMotor);

    CTREUtil.attempt(
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                _floorVelocityGetter,
                _floorMotor.getSupplyCurrent(),
                _floorMotor.getStatorCurrent()),
        _floorMotor);

    CTREUtil.attempt(
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                _feedVelocityGetter,
                _feedMotor.getSupplyCurrent(),
                _feedMotor.getStatorCurrent()),
        _feedMotor);

    FaultLogger.register(_floorMotor);
    FaultLogger.register(_feedMotor);

    // EVERYTHING BELOW IS TEMPORARY
    DogLog.tunable(
        "Desired Floor Speed RPS", 0.0, newRps -> _desiredFloorSpeed.mut_setMagnitude(newRps));
    DogLog.tunable(
        "Desired Feed Speed RPS", 0.0, newRps -> _desiredFeedSpeed.mut_setMagnitude(newRps));

    final CoastOut coast = new CoastOut();

    setDefaultCommand(
        run(
            () -> {
              _floorMotor.setControl(coast);
              _feedMotor.setControl(coast);
            }));
  } 

    private void setFloorSpeed(AngularVelocity desiredFloorSpeed) {
    double errorRps = desiredFloorSpeed.minus(getFloorSpeed()).in(RotationsPerSecond);

    if (Math.abs(errorRps) > velocityThreshold.in(RotationsPerSecond)) {
      _floorMotor.setControl(_dutyCycleSetter.withOutput(Math.signum(errorRps)));
    } else {
      _floorMotor.setControl(_velocitySetter.withVelocity(desiredFloorSpeed));
    }
  }

  private void setFeedSpeed(AngularVelocity desiredFeedSpeed) {
    double errorRps = desiredFeedSpeed.minus(getFeedSpeed()).in(RotationsPerSecond);

    if (Math.abs(errorRps) > velocityThreshold.in(RotationsPerSecond)) {
      _feedMotor.setControl(_dutyCycleSetter.withOutput(Math.signum(errorRps)));
    } else {
      _feedMotor.setControl(_velocitySetter.withVelocity(desiredFeedSpeed));
    }
  }

    public Command index() {
    return run(() -> {
          setFloorSpeed(_desiredFloorSpeed);
          setFeedSpeed(_desiredFeedSpeed);
        })
        .withName("index");
  }

  @Logged(name = "Floor Speed")
  public AngularVelocity getFloorSpeed() {
    return _floorVelocityGetter.refresh().getValue();
  }

  @Logged(name = "Feed Speed")
  public AngularVelocity getFeedSpeed() {
    return _feedVelocityGetter.refresh().getValue();
  }

  @Override
  public void close() {
    _floorMotor.close();
    _feedMotor.close();
  }

}
