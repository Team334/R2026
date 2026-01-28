// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AdvancedSubsystem;
import frc.lib.CTREUtil;
import frc.lib.FaultLogger;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;

public class Climb extends AdvancedSubsystem {
  private final TalonFX _climbMotor = new TalonFX(ClimbConstants.climbMotorID, Constants.subsystemBus);

  private final MotionMagicVoltage _heightSetter = new MotionMagicVoltage(0);

  private final StatusSignal<Angle> _heightGetter = _climbMotor.getPosition();
  
  public Climb() {
    var climbMotorConfigs = new TalonFXConfiguration();
    var climbSlot = new SlotConfigs();

    climbSlot.kS = ClimbConstants.climbkS.in(Volts);
    climbSlot.kG = ClimbConstants.climbkG.in(Volts);
    climbSlot.kV = ClimbConstants.climbkV.in(Volts.per(RotationsPerSecond));
    climbSlot.kA = ClimbConstants.climbkA.in(Volts.per(RotationsPerSecondPerSecond));

    climbSlot.kP = ClimbConstants.climbkP.in(Volts.per(Rotations));

    climbMotorConfigs.Slot0 = Slot0Configs.from(climbSlot);

    climbMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    climbMotorConfigs.Feedback.SensorToMechanismRatio = ClimbConstants.climbGearRatio;

    climbMotorConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        ClimbConstants.maxElevatorHeight.in(Rotations);
    climbMotorConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        ClimbConstants.minElevatorHeight.in(Rotations);

    climbMotorConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    climbMotorConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    CTREUtil.attempt(() -> _climbMotor.getConfigurator().apply(climbMotorConfigs), _climbMotor);
    CTREUtil.attempt(() -> _climbMotor.optimizeBusUtilization(), _climbMotor);

    FaultLogger.register(_climbMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void setHeight(Angle height) {
    _climbMotor.setControl(_heightSetter.withPosition(height.in(Rotations)));
  }

  public Command extend() {
    return run(() -> setHeight(ClimbConstants.maxElevatorHeight)).withName("Extend Climb");
  }

  public Command retract() {
    return run(() -> setHeight(ClimbConstants.minElevatorHeight)).withName("Retract Climb");
  }

  public Command goToHeight(Angle target) {
    return run(() -> setHeight(target)).withName("Climb to Position");
  }

  @Logged(name = "Climb Height")
  public Angle getHeight() {
    return _heightGetter.refresh().getValue();
  }

  @Override
  public void close() {
    _climbMotor.close();
  }
}
