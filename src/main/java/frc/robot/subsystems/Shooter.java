package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AdvancedSubsystem;
import frc.lib.CTREUtil;
import frc.lib.FaultLogger;
import frc.robot.Constants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShotPresets;
import frc.robot.Robot;
import frc.robot.utils.ShotPreset;
import java.util.function.Supplier;

public class Shooter extends AdvancedSubsystem {
  private final TalonFX _flywheelMotor =
      new TalonFX(ShooterConstants.flywheelMotorID, Constants.subsystemBus);
  private final TalonFX _flywheelFollowerMotor =
      new TalonFX(ShooterConstants.flywheelFollowerMotorID, Constants.subsystemBus);
  private final TalonFX _hoodMotor =
      new TalonFX(ShooterConstants.hoodMotorID, Constants.subsystemBus);

  private final VelocityVoltage _flywheelVelocitySetter = new VelocityVoltage(0);
  private final DutyCycleOut _flywheelDutyCycleSetter = new DutyCycleOut(0);

  private final MotionMagicVoltage _hoodAngleSetter = new MotionMagicVoltage(0);

  private final StatusSignal<AngularVelocity> _flywheelVelocityGetter =
      _flywheelMotor.getVelocity();

  private final StatusSignal<Angle> _hoodAngleGetter = _hoodMotor.getPosition();

  @Logged(name = "Velocity Threshold")
  private final AngularVelocity velocityThreshold = RotationsPerSecond.of(3);

  @Logged(name = "Idle Velocity Percentage")
  private final double idleVelocityPercentage = 0.5;

  private final Supplier<ShotPreset> _shotPresetSupplier;

  private DCMotorSim _flywheelSim;
  private DCMotorSim _hoodSim;

  private Notifier _simNotifier;
  private double _lastSimTime;

  public Shooter(Supplier<ShotPreset> shotPresetSupplier) {
    _shotPresetSupplier = shotPresetSupplier;

    var flywheelMotorConfigs = new TalonFXConfiguration();
    var flywheelFollowerMotorConfigs = new TalonFXConfiguration();
    var hoodMotorConfigs = new TalonFXConfiguration();

    // flywheel motor configs
    flywheelMotorConfigs.CurrentLimits.StatorCurrentLimit = 100;
    flywheelMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

    flywheelMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = false;

    flywheelMotorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    flywheelMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    flywheelMotorConfigs.Slot0.kS = ShooterConstants.flywheelkS.in(Volts);
    flywheelMotorConfigs.Slot0.kV = ShooterConstants.flywheelkV.in(Volts.per(RotationsPerSecond));

    flywheelMotorConfigs.Slot0.kP = ShooterConstants.flywheelkP.in(Volts.per(RotationsPerSecond));

    flywheelMotorConfigs.Feedback.SensorToMechanismRatio = ShooterConstants.flywheelGearRatio;

    // flywheel follower motor configs
    flywheelFollowerMotorConfigs.CurrentLimits.StatorCurrentLimit = 100;
    flywheelFollowerMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

    flywheelFollowerMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = false;

    flywheelFollowerMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // hood motor configs
    hoodMotorConfigs.Slot0.kS = ShooterConstants.hoodkS.in(Volts);
    hoodMotorConfigs.Slot0.kG = ShooterConstants.hoodkG.in(Volts);
    hoodMotorConfigs.Slot0.kV = ShooterConstants.hoodkV.in(Volts.per(RotationsPerSecond));
    hoodMotorConfigs.Slot0.kA = ShooterConstants.hoodkA.in(Volts.per(RotationsPerSecondPerSecond));

    hoodMotorConfigs.Slot0.kP = ShooterConstants.hoodkP.in(Volts.per(Rotations));

    hoodMotorConfigs.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    hoodMotorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    hoodMotorConfigs.Feedback.SensorToMechanismRatio = ShooterConstants.hoodGearRatio;

    hoodMotorConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        ShooterConstants.hoodForwardSoftLimitThreshold.in(Rotations);
    hoodMotorConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        ShooterConstants.hoodReverseSoftLimitThreshold.in(Rotations);

    hoodMotorConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    hoodMotorConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    hoodMotorConfigs.MotionMagic.MotionMagicCruiseVelocity =
        ShooterConstants.hoodVelocity.in(RotationsPerSecond);
    hoodMotorConfigs.MotionMagic.MotionMagicAcceleration =
        ShooterConstants.hoodAcceleration.in(RotationsPerSecondPerSecond);

    CTREUtil.attempt(
        () -> _flywheelMotor.getConfigurator().apply(flywheelMotorConfigs), _flywheelMotor);
    CTREUtil.attempt(
        () -> _flywheelFollowerMotor.getConfigurator().apply(flywheelFollowerMotorConfigs),
        _flywheelFollowerMotor);
    CTREUtil.attempt(() -> _hoodMotor.getConfigurator().apply(hoodMotorConfigs), _hoodMotor);

    CTREUtil.attempt(() -> _flywheelMotor.optimizeBusUtilization(), _flywheelMotor);
    CTREUtil.attempt(() -> _flywheelFollowerMotor.optimizeBusUtilization(), _flywheelFollowerMotor);
    CTREUtil.attempt(() -> _hoodMotor.optimizeBusUtilization(), _hoodMotor);

    CTREUtil.attempt(
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                _flywheelVelocityGetter,
                _flywheelMotor.getSupplyCurrent(),
                _flywheelMotor.getStatorCurrent()),
        _flywheelMotor);

    FaultLogger.register(_flywheelMotor);
    FaultLogger.register(_flywheelFollowerMotor);
    FaultLogger.register(_hoodMotor);

    _flywheelFollowerMotor.setControl(
        new Follower(ShooterConstants.flywheelMotorID, MotorAlignmentValue.Opposed));

    setDefaultCommand(idle());

    if (Robot.isSimulation()) {
      var flywheelMotorSimConfigs = new TalonFXConfiguration();
      var hoodMotorSimConfigs = new TalonFXConfiguration();

      _flywheelMotor.getConfigurator().refresh(flywheelMotorSimConfigs);
      _hoodMotor.getConfigurator().refresh(hoodMotorSimConfigs);

      flywheelMotorSimConfigs.Slot0.kS = 0;

      hoodMotorSimConfigs.Slot0.kS = 0;
      hoodMotorSimConfigs.Slot0.kG = 0;

      _hoodMotor.setPosition(0);
      hoodMotorSimConfigs.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);

      _flywheelMotor.getConfigurator().apply(flywheelMotorSimConfigs);
      _hoodMotor.getConfigurator().apply(hoodMotorSimConfigs);

      _flywheelSim =
          new DCMotorSim(
              LinearSystemId.createDCMotorSystem(
                  MotorConstants.krakenX44, 0.001, ShooterConstants.flywheelGearRatio),
              MotorConstants.krakenX44);

      _hoodSim =
          new DCMotorSim(
              LinearSystemId.createDCMotorSystem(
                  ShooterConstants.hoodkV.in(Volts.per(RadiansPerSecond)),
                  ShooterConstants.hoodkA.in(Volts.per(RadiansPerSecondPerSecond))),
              MotorConstants.krakenX44);

      startSimThread();
    }
  }

  private void startSimThread() {
    _lastSimTime = Utils.getCurrentTimeSeconds();

    _simNotifier =
        new Notifier(
            () -> {
              final double batteryVolts = RobotController.getBatteryVoltage();

              final double currentTime = Utils.getCurrentTimeSeconds();
              final double deltaTime = currentTime - _lastSimTime;

              var flywheelMotorSimState = _flywheelMotor.getSimState();

              flywheelMotorSimState.setSupplyVoltage(batteryVolts);

              _flywheelSim.setInputVoltage(
                  flywheelMotorSimState.getMotorVoltageMeasure().in(Volts));
              _flywheelSim.update(deltaTime);

              flywheelMotorSimState.setRawRotorPosition(
                  _flywheelSim.getAngularPosition().times(ShooterConstants.flywheelGearRatio));
              flywheelMotorSimState.setRotorVelocity(
                  _flywheelSim.getAngularVelocity().times(ShooterConstants.flywheelGearRatio));

              var hoodMotorSimState = _hoodMotor.getSimState();

              hoodMotorSimState.setSupplyVoltage(batteryVolts);

              _hoodSim.setInputVoltage(hoodMotorSimState.getMotorVoltageMeasure().in(Volts));
              _hoodSim.update(deltaTime);

              hoodMotorSimState.setRawRotorPosition(
                  _hoodSim.getAngularPosition().times(ShooterConstants.hoodGearRatio));
              hoodMotorSimState.setRotorVelocity(
                  _hoodSim.getAngularVelocity().times(ShooterConstants.hoodGearRatio));

              _lastSimTime = currentTime;
            });

    _simNotifier.setName("Shooter Sim Thread");
    _simNotifier.startPeriodic(1 / Constants.simNotifierFrequency.in(Hertz));
  }

  private void setFlywheelSpeed(AngularVelocity desiredFrontSpeed) {
    double errorRps = desiredFrontSpeed.minus(getFlywheelSpeed()).in(RotationsPerSecond);

    if (Math.abs(errorRps) > velocityThreshold.in(RotationsPerSecond)) {
      _flywheelMotor.setControl(_flywheelDutyCycleSetter.withOutput(Math.signum(errorRps)));
    } else {
      _flywheelMotor.setControl(_flywheelVelocitySetter.withVelocity(desiredFrontSpeed));
    }
  }

  private void setHoodAngle(Angle angle) {
    _hoodMotor.setControl(_hoodAngleSetter.withPosition(angle));
  }

  /** Set hood to shooting angle, flywheels to {@link #idleVelocityPercentage} of shooting speed. */
  public Command idle() {
    return run(() -> {
          ShotPreset preset = _shotPresetSupplier.get();

          setFlywheelSpeed(preset.getFlywheelSpeed().times(idleVelocityPercentage));
          setHoodAngle(preset.getHoodAngle());
        })
        .withName("Idle");
  }

  /** Scores / ferries depending on robot pose. */
  public Command shoot() {
    return run(() -> {
          ShotPreset preset = _shotPresetSupplier.get();

          setFlywheelSpeed(preset.getFlywheelSpeed());
          setHoodAngle(preset.getHoodAngle());
        })
        .withName("Shoot");
  }

  /** Spits the fuel in front of the robot at a fixed angle and speed. */
  public Command spit() {
    return run(() -> {
          setFlywheelSpeed(ShotPresets.spitPreset.getFlywheelSpeed());
          setHoodAngle(ShotPresets.spitPreset.getHoodAngle());
        })
        .withName("Spit");
  }

  @Logged(name = "Flywheel Speed")
  public AngularVelocity getFlywheelSpeed() {
    return _flywheelVelocityGetter.refresh().getValue();
  }

  @Logged(name = "Hood Angle")
  public Angle getHoodAngle() {
    return _hoodAngleGetter.refresh().getValue();
  }

  @Override
  public void periodic() {
    DogLog.time("Timing/Shooter/periodic()");
    super.periodic();
    DogLog.timeEnd("Timing/Shooter/periodic()");
  }

  @Override
  public void close() {
    _flywheelMotor.close();
    _flywheelFollowerMotor.close();
    _hoodMotor.close();
  }
}
