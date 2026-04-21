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
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.fault.FaultLogger;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.lib.util.CTREUtil;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Robot;

public class IntakeFeed extends AdvancedSubsystem {
  private final TalonFX _feedMotor =
      new TalonFX(IntakeConstants.feedMotorID, Constants.subsystemBus);

  private final VelocityVoltage _feedVelocitySetter = new VelocityVoltage(0);
  private final StatusSignal<AngularVelocity> _feedVelocityGetter = _feedMotor.getVelocity();

  private DCMotorSim _feedSim;

  private Notifier _simNotifier;
  private double _lastSimTime;

  public IntakeFeed() {
    var feedMotorConfigs = new TalonFXConfiguration();

    // feed motor configs
    feedMotorConfigs.Slot0.kS = IntakeConstants.feedkS.in(Volts);
    feedMotorConfigs.Slot0.kV = IntakeConstants.feedkV.in(Volts.per(RotationsPerSecond));

    feedMotorConfigs.Slot0.kP = IntakeConstants.feedkP.in(Volts.per(RotationsPerSecond));

    feedMotorConfigs.CurrentLimits.SupplyCurrentLimit = IntakeConstants.feedSupplyLimit.in(Amps);
    feedMotorConfigs.CurrentLimits.SupplyCurrentLowerTime = 0;

    feedMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

    feedMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    feedMotorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    feedMotorConfigs.Feedback.SensorToMechanismRatio = IntakeConstants.feedGearRatio;

    CTREUtil.attempt(() -> _feedMotor.getConfigurator().apply(feedMotorConfigs), _feedMotor);
    CTREUtil.attempt(() -> _feedMotor.optimizeBusUtilization(), _feedMotor);

    FaultLogger.register(_feedMotor);

    if (Robot.isSimulation()) {
      var feedMotorSimConfigs = new TalonFXConfiguration();

      _feedMotor.getConfigurator().refresh(feedMotorSimConfigs);

      feedMotorSimConfigs.Slot0.kS = 0;

      _feedMotor.getConfigurator().apply(feedMotorSimConfigs);

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
        .withName("Feed In");
  }

  /** Runs the feed wheels outwards */
  public Command feedOut() {
    return run(() ->
            _feedMotor.setControl(
                _feedVelocitySetter.withVelocity(IntakeConstants.feedSpeed.unaryMinus())))
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
