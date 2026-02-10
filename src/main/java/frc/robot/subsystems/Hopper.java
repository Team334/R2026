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

  private final TalonFX _floorMotor =
      new TalonFX(HopperConstants.floorMotorID, Constants.subsystemBus);

  private final VelocityVoltage _rollerVelocitySetter = new VelocityVoltage(0);
  private final VelocityVoltage _floorVelocitySetter = new VelocityVoltage(0);

  private final StatusSignal<AngularVelocity> _rollerVelocityGetter = _rollerMotor.getVelocity();
  private final StatusSignal<AngularVelocity> _floorVelocityGetter = _floorMotor.getVelocity();

  public Hopper() {
    var rollerMotorConfig = new TalonFXConfiguration();
    var floorMotorConfig = new TalonFXConfiguration();

    // roller motor configs
    rollerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    rollerMotorConfig.Slot0.kS = HopperConstants.rollerkS.in(Volts);
    rollerMotorConfig.Slot0.kV = HopperConstants.rollerkV.in(Volts.per(RotationsPerSecond));

    rollerMotorConfig.Slot0.kP = HopperConstants.rollerkP.in(Volts.per(RotationsPerSecond));

    rollerMotorConfig.Feedback.SensorToMechanismRatio = HopperConstants.rollerGearRatio;

    // floor motor configs
    floorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    floorMotorConfig.Slot0.kS = HopperConstants.floorkS.in(Volts);
    floorMotorConfig.Slot0.kV = HopperConstants.floorkV.in(Volts.per(RotationsPerSecond));

    floorMotorConfig.Slot0.kP = HopperConstants.floorkP.in(Volts.per(RotationsPerSecond));

    floorMotorConfig.Feedback.SensorToMechanismRatio = HopperConstants.floorGearRatio;

    CTREUtil.attempt(() -> _rollerMotor.getConfigurator().apply(rollerMotorConfig), _rollerMotor);
    CTREUtil.attempt(() -> _floorMotor.getConfigurator().apply(floorMotorConfig), _floorMotor);

    CTREUtil.attempt(() -> _rollerMotor.optimizeBusUtilization(), _rollerMotor);
    CTREUtil.attempt(() -> _floorMotor.optimizeBusUtilization(), _floorMotor);

    FaultLogger.register(_rollerMotor);
    FaultLogger.register(_floorMotor);

    setDefaultCommand(
        run(
            () -> {
              _floorMotor.setControl(_floorVelocitySetter.withVelocity(0));
              _rollerMotor.setControl(_rollerVelocitySetter.withVelocity(0));
            }));
  }

  /** Feeds fuel into shooter. */
  public Command feed() {
    return run(() -> {
          _floorMotor.setControl(
              _floorVelocitySetter.withVelocity(
                  HopperConstants.floorFeedSpeed.in(RotationsPerSecond)));
          _rollerMotor.setControl(
              _rollerVelocitySetter.withVelocity(
                  HopperConstants.rollerFeedSpeed.in(RotationsPerSecond)));
        })
        .withName("Fee");
  }

  @Logged(name = "Roller Speed")
  public AngularVelocity getRollerSpeed() {
    return _rollerVelocityGetter.refresh().getValue();
  }

  @Logged(name = "Floor Speed")
  public AngularVelocity getFloorSpeed() {
    return _floorVelocityGetter.refresh().getValue();
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
    _floorMotor.close();
  }
}
