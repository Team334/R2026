// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.AdvancedSubsystem;
import frc.lib.CTREUtil;
import frc.lib.FaultLogger;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Robot;

public class IntakeFeed extends AdvancedSubsystem {
  private final TalonFX _feedMotor =
      new TalonFX(IntakeConstants.feedMotorID, Constants.subsystemBus);

  private final VelocityVoltage _feedVelocitySetter = new VelocityVoltage(0);
  private final StatusSignal<AngularVelocity> _feedVelocityGetter = _feedMotor.getVelocity();

  private final Trigger _intakeLowered;

  private DCMotorSim _feedSim;

  private Notifier _simNotifier;
  private double _lastSimTime;

  public IntakeFeed(Trigger intakeLowered) {
    _intakeLowered = intakeLowered;

    var feedMotorConfigs = new TalonFXConfiguration();

    // feed motor configs
    feedMotorConfigs.Slot0.kS = IntakeConstants.feedkS.in(Volts);
    feedMotorConfigs.Slot0.kV = IntakeConstants.feedkV.in(Volts.per(RotationsPerSecond));

    feedMotorConfigs.Slot0.kP = IntakeConstants.feedkP.in(Volts.per(RotationsPerSecond));

    feedMotorConfigs.Feedback.SensorToMechanismRatio = IntakeConstants.feedGearRatio;

    CTREUtil.attempt(() -> _feedMotor.getConfigurator().apply(feedMotorConfigs), _feedMotor);
    CTREUtil.attempt(() -> _feedMotor.optimizeBusUtilization(), _feedMotor);

    FaultLogger.register(_feedMotor);

    if (Robot.isSimulation()) {
      var c = new TalonFXConfiguration();

      _feedMotor.getConfigurator().refresh(c);

      c.Slot0.kS = 0;

      _feedMotor.getConfigurator().apply(c);

      _feedSim =
          new DCMotorSim(
              LinearSystemId.createDCMotorSystem(
                  MotorConstants.krakenX44, 0.001, IntakeConstants.feedGearRatio),
              MotorConstants.krakenX44);

      _feedVelocitySetter.withUpdateFreqHz(Hertz.of(1000));
      _feedVelocityGetter.setUpdateFrequency(Hertz.of(1000));

      startSimThread();
    }

    setDefaultCommand(feedStop());
  }

  private void startSimThread() {
    _lastSimTime = Utils.getCurrentTimeSeconds();

    _simNotifier =
        new Notifier(
            () -> {
              final double batteryVolts = RobotController.getBatteryVoltage();

              final double currentTime = Utils.getCurrentTimeSeconds();
              final double deltaTime = currentTime - _lastSimTime;

              var feedMotorSimState = _feedMotor.getSimState();

              feedMotorSimState.setSupplyVoltage(batteryVolts);

              _feedSim.setInputVoltage(feedMotorSimState.getMotorVoltageMeasure().in(Volts));
              _feedSim.update(deltaTime);

              feedMotorSimState.setRawRotorPosition(
                  _feedSim.getAngularPosition().times(IntakeConstants.feedGearRatio));
              feedMotorSimState.setRotorVelocity(
                  _feedSim.getAngularVelocity().times(IntakeConstants.feedGearRatio));

              _lastSimTime = currentTime;
            });

    _simNotifier.setName("IntakeFeed Sim Thread");
    _simNotifier.startPeriodic(1 / Constants.simNotifierFrequency.in(Hertz));
  }

  @Logged(name = "Speed")
  public AngularVelocity getSpeed() {
    return _feedVelocityGetter.refresh().getValue();
  }

  /** Runs the feed wheels inwards */
  public Command feedIn() {
    return run(() ->
            _feedMotor.setControl(_feedVelocitySetter.withVelocity(IntakeConstants.feedSpeed)))
        .onlyIf(_intakeLowered)
        .until(_intakeLowered.negate())
        .withName("Feed In");
  }

  /** Runs the feed wheels outwards */
  public Command feedOut() {
    return run(() ->
            _feedMotor.setControl(
                _feedVelocitySetter.withVelocity(IntakeConstants.feedSpeed.unaryMinus())))
        .onlyIf(_intakeLowered)
        .until(_intakeLowered.negate())
        .withName("Feed Out");
  }

  /** Stops the feed wheels */
  public Command feedStop() {
    return run(() -> _feedMotor.setControl(_feedVelocitySetter.withVelocity(0)))
        .withName("Feed Stop");
  }

  @Override
  public void periodic() {
    DogLog.time("Timing/IntakeFeed/periodic()");
    super.periodic();
    DogLog.timeEnd("Timing/IntakeFeed/periodic()");
  }

  @Override
  public void close() {
    _feedMotor.close();
  }
}
