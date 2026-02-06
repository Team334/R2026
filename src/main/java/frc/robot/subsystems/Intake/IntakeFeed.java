// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AdvancedSubsystem;
import frc.lib.CTREUtil;
import frc.lib.FaultLogger;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeFeed extends AdvancedSubsystem {
  private final TalonFX _feedMotor =
      new TalonFX(IntakeConstants.feedMotorID, Constants.subsystemBus);

  private final VelocityVoltage _feedVelocitySetter = new VelocityVoltage(0);
  private final StatusSignal<AngularVelocity> _feedVelocityGetter = _feedMotor.getVelocity();

  public IntakeFeed() {
    var feedMotorConfigs = new TalonFXConfiguration();

    // feed motor configs
    feedMotorConfigs.Slot0.kS = IntakeConstants.feedkS.in(Volts);
    feedMotorConfigs.Slot0.kV = IntakeConstants.feedkV.in(Volts.per(RotationsPerSecond));

    feedMotorConfigs.Slot0.kP = IntakeConstants.feedkP.in(Volts.per(RotationsPerSecond));

    feedMotorConfigs.Feedback.SensorToMechanismRatio = IntakeConstants.feedGearRatio;

    CTREUtil.attempt(() -> _feedMotor.getConfigurator().apply(feedMotorConfigs), _feedMotor);
    CTREUtil.attempt(() -> _feedMotor.optimizeBusUtilization(), _feedMotor);

    FaultLogger.register(_feedMotor);

    setDefaultCommand(feedStop());
  }

  @Logged(name = "Speed")
  public AngularVelocity getSpeed() {
    return _feedVelocityGetter.refresh().getValue();
  }

  /** Runs the feed wheels inwards */
  public Command feedIn() {
    return run(
        () -> _feedMotor.setControl(_feedVelocitySetter.withVelocity(IntakeConstants.feedSpeed)));
  }

  /** Runs the feed wheels outwards */
  public Command feedOut() {
    return run(
        () ->
            _feedMotor.setControl(
                _feedVelocitySetter.withVelocity(IntakeConstants.feedSpeed.unaryMinus())));
  }

  /** Stops the feed wheels */
  public Command feedStop() {
    return run(() -> _feedMotor.setControl(_feedVelocitySetter.withVelocity(0)));
  }

  @Override
  public void periodic() {
    DogLog.time("Time/IntakeFeed/periodic()");
    super.periodic();
    DogLog.timeEnd("Time/IntakeFeed/periodic()");
  }

  @Override
  public void close() {
    _feedMotor.close();
  }
}
