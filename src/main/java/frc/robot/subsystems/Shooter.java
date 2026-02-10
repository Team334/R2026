package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AdvancedSubsystem;
import frc.lib.CTREUtil;
import frc.lib.FaultLogger;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShotPresets;
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

  public Shooter(Supplier<ShotPreset> shotPresetSupplier) {
    _shotPresetSupplier = shotPresetSupplier;

    var flywheelMotorConfig = new TalonFXConfiguration();
    var flywheelFollowerMotorConfig = new TalonFXConfiguration();
    var hoodMotorConfig = new TalonFXConfiguration();

    // flywheel motor configs
    flywheelMotorConfig.CurrentLimits.StatorCurrentLimit = 100;
    flywheelMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    flywheelMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = false;

    flywheelMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    flywheelMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    flywheelMotorConfig.Slot0.kS = ShooterConstants.flywheelkS.in(Volts);
    flywheelMotorConfig.Slot0.kV = ShooterConstants.flywheelkV.in(Volts.per(RotationsPerSecond));

    flywheelMotorConfig.Slot0.kP = ShooterConstants.flywheelkP.in(Volts.per(RotationsPerSecond));

    flywheelMotorConfig.Feedback.SensorToMechanismRatio = ShooterConstants.flywheelGearRatio;

    // flywheel follower motor configs
    flywheelFollowerMotorConfig.CurrentLimits.StatorCurrentLimit = 100;
    flywheelFollowerMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    flywheelFollowerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = false;

    flywheelFollowerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // hood motor configs
    hoodMotorConfig.Slot0.kS = ShooterConstants.hoodkS.in(Volts);
    hoodMotorConfig.Slot0.kG = ShooterConstants.hoodkG.in(Volts);
    hoodMotorConfig.Slot0.kV = ShooterConstants.hoodkV.in(Volts.per(RotationsPerSecond));
    hoodMotorConfig.Slot0.kA = ShooterConstants.hoodkA.in(Volts.per(RotationsPerSecondPerSecond));

    hoodMotorConfig.Slot0.kP = ShooterConstants.hoodkP.in(Volts.per(Rotations));

    hoodMotorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    hoodMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    hoodMotorConfig.Feedback.SensorToMechanismRatio = ShooterConstants.hoodGearRatio;

    hoodMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        ShooterConstants.hoodForwardSoftLimitThreshold.in(Rotations);
    hoodMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        ShooterConstants.hoodReverseSoftLimitThreshold.in(Rotations);

    hoodMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    hoodMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    hoodMotorConfig.MotionMagic.MotionMagicCruiseVelocity =
        ShooterConstants.hoodVelocity.in(RotationsPerSecond);
    hoodMotorConfig.MotionMagic.MotionMagicAcceleration =
        ShooterConstants.hoodAcceleration.in(RotationsPerSecondPerSecond);

    CTREUtil.attempt(
        () -> _flywheelMotor.getConfigurator().apply(flywheelMotorConfig), _flywheelMotor);
    CTREUtil.attempt(
        () -> _flywheelFollowerMotor.getConfigurator().apply(flywheelFollowerMotorConfig),
        _flywheelFollowerMotor);
    CTREUtil.attempt(() -> _hoodMotor.getConfigurator().apply(hoodMotorConfig), _hoodMotor);

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
