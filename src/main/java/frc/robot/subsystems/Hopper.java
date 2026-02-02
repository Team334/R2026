// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AdvancedSubsystem;
import frc.lib.CTREUtil;
import frc.lib.FaultLogger;
import frc.robot.Constants;
import frc.robot.Constants.HopperConstants;

public class Hopper extends AdvancedSubsystem {
  private final TalonFX _rollerMotor =
      new TalonFX(HopperConstants.rollerMotorID, Constants.subsystemBus);

  private final TalonFX _feedMotor =
      new TalonFX(HopperConstants.feedMotorID, Constants.subsystemBus);

  private final VelocityVoltage _rollerVelocitySetter = new VelocityVoltage(0);
  private final VelocityVoltage _feedVelocitySetter = new VelocityVoltage(0);

  private final StatusSignal<AngularVelocity> _rollerVelocityGetter = _rollerMotor.getVelocity();
  private final StatusSignal<AngularVelocity> _feedVelocityGetter = _feedMotor.getVelocity();

  public Hopper() {
    var rollerMotorConfig = new TalonFXConfiguration();
    var feedMotorConfig = new TalonFXConfiguration();

    // Roller motor configs
    rollerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    rollerMotorConfig.Slot0.kS = HopperConstants.rollerkS.in(Volts);
    rollerMotorConfig.Slot0.kV = HopperConstants.rollerkV.in(Volts.per(RotationsPerSecond));

    rollerMotorConfig.Slot0.kP = HopperConstants.rollerkP.in(Volts.per(RotationsPerSecond));

    rollerMotorConfig.Feedback.SensorToMechanismRatio = HopperConstants.rollerGearRatio;

    // feed motor configs
    feedMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    feedMotorConfig.Slot0.kS = HopperConstants.feedkS.in(Volts);
    feedMotorConfig.Slot0.kV = HopperConstants.feedkV.in(Volts.per(RotationsPerSecond));

    feedMotorConfig.Slot0.kP = HopperConstants.feedkP.in(Volts.per(RotationsPerSecond));

    feedMotorConfig.Feedback.SensorToMechanismRatio = HopperConstants.feedGearRatio;

    CTREUtil.attempt(() -> _rollerMotor.getConfigurator().apply(rollerMotorConfig), _rollerMotor);
    CTREUtil.attempt(() -> _feedMotor.getConfigurator().apply(feedMotorConfig), _feedMotor);

    CTREUtil.attempt(() -> _rollerMotor.optimizeBusUtilization(), _rollerMotor);
    CTREUtil.attempt(() -> _feedMotor.optimizeBusUtilization(), _feedMotor);

    FaultLogger.register(_rollerMotor);
    FaultLogger.register(_feedMotor);

    setDefaultCommand(
        run(
            () -> {
              _feedMotor.setControl(_feedVelocitySetter.withVelocity(0));
              _rollerMotor.setControl(_rollerVelocitySetter.withVelocity(0));
            }));
  }

  /** Index fuel into the shooter. */
  public Command index() {
    return run(() -> {
          _feedMotor.setControl(
              _feedVelocitySetter.withVelocity(
                  HopperConstants.feedIndexSpeed.in(RotationsPerSecond)));
          _rollerMotor.setControl(
              _rollerVelocitySetter.withVelocity(
                  HopperConstants.rollerIndexSpeed.in(RotationsPerSecond)));
        })
        .withName("Index");
  }

  @Logged(name = "Roller Speed")
  public AngularVelocity getRollerSpeed() {
    return _rollerVelocityGetter.refresh().getValue();
  }

  @Logged(name = "Feed Speed")
  public AngularVelocity getFeedSpeed() {
    return _feedVelocityGetter.refresh().getValue();
  }

  @Override
  public void periodic() {
    DogLog.time("Timing/Hopper/periodic()");
    super.periodic();
    DogLog.timeEnd("Timing/Hopper/periodic()");
  }

  @Override
  public void close() {
    _rollerMotor.close();
    _feedMotor.close();
  }
}
