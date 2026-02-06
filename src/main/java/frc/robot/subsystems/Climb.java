// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AdvancedSubsystem;
import frc.lib.CTREUtil;
import frc.lib.FaultLogger;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;

public class Climb extends AdvancedSubsystem {
  private final TalonFX _climbMotor =
      new TalonFX(ClimbConstants.climbMotorID, Constants.subsystemBus);

  private final MotionMagicVoltage _heightSetter = new MotionMagicVoltage(0);
  private final StatusSignal<Angle> _heightGetter = _climbMotor.getPosition();

  public Climb() {
    var climbMotorConfigs = new TalonFXConfiguration();
    var climbSlotConfigs = new SlotConfigs();

    // climb motor configs
    climbSlotConfigs.kS = ClimbConstants.kS.in(Volts);
    climbSlotConfigs.kG = ClimbConstants.kG.in(Volts);
    climbSlotConfigs.kV = ClimbConstants.kV.in(Volts.per(RotationsPerSecond));

    climbSlotConfigs.kA = ClimbConstants.kA.in(Volts.per(RotationsPerSecondPerSecond));
    climbSlotConfigs.kP = ClimbConstants.kP.in(Volts.per(Rotation));

    climbMotorConfigs.Slot0 = Slot0Configs.from(climbSlotConfigs);
    climbMotorConfigs.Slot1 =
        Slot1Configs.from(
            climbSlotConfigs
                .withKG(ClimbConstants.climbingkG.in(Volts))
                .withKA(ClimbConstants.climbingkA.in(Volts.per(RotationsPerSecondPerSecond)))
                .withKP(ClimbConstants.climbingkP.in(Volts.per(Rotation))));

    climbMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    climbMotorConfigs.Feedback.SensorToMechanismRatio = ClimbConstants.climbGearRatio;

    climbMotorConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        ClimbConstants.extended.in(Rotations);
    climbMotorConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        ClimbConstants.retracted.in(Rotations);

    climbMotorConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    climbMotorConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    CTREUtil.attempt(() -> _climbMotor.getConfigurator().apply(climbMotorConfigs), _climbMotor);
    CTREUtil.attempt(() -> _climbMotor.optimizeBusUtilization(), _climbMotor);

    FaultLogger.register(_climbMotor);
  }

  /** Extends the climb. */
  public Command extend() {
    return run(() ->
            _climbMotor.setControl(
                _heightSetter.withPosition(ClimbConstants.extended.in(Rotations)).withSlot(0)))
        .withName("Extend");
  }

  /** Retracts the climb under no load. */
  public Command retract() {
    return run(() ->
            _climbMotor.setControl(
                _heightSetter.withPosition(ClimbConstants.retracted.in(Rotations)).withSlot(0)))
        .withName("Retract");
  }

  /** Retracts the climb under the robot's weight. */
  public Command climb() {
    return run(() ->
            _climbMotor.setControl(
                _heightSetter.withPosition(ClimbConstants.retracted.in(Rotations)).withSlot(1)))
        .withName("Climb");
  }

  @Logged(name = "Height")
  public Angle getHeight() {
    return _heightGetter.refresh().getValue();
  }

  @Override
  public void periodic() {
    DogLog.time("Time/Climb/periodic()");
    super.periodic();
    DogLog.timeEnd("Time/Climb/periodic()");
  }

  @Override
  public void close() {
    _climbMotor.close();
  }
}
