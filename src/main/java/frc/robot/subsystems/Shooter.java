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

public class Shooter extends AdvancedSubsystem {
  private final TalonFX _frontMotor =
      new TalonFX(ShooterConstants.frontMotorID, Constants.subsystemBus);
  private final TalonFX _frontFollowerMotor =
      new TalonFX(ShooterConstants.frontFollowerMotorID, Constants.subsystemBus);
  private final TalonFX _backMotor =
      new TalonFX(ShooterConstants.backMotorID, Constants.subsystemBus);
  private final TalonFX _hoodMotor =
      new TalonFX(ShooterConstants.hoodMotorID, Constants.subsystemBus);

  private final VelocityVoltage _frontVelocitySetter = new VelocityVoltage(0);
  private final DutyCycleOut _frontDutyCycleSetter = new DutyCycleOut(0);

  private final VelocityVoltage _backVelocitySetter = new VelocityVoltage(0);
  private final DutyCycleOut _backDutyCycleSetter = new DutyCycleOut(0);

  private final MotionMagicVoltage _hoodAngleSetter = new MotionMagicVoltage(0);

  private final StatusSignal<AngularVelocity> _frontVelocityGetter = _frontMotor.getVelocity();
  private final StatusSignal<AngularVelocity> _backVelocityGetter = _backMotor.getVelocity();
  private final StatusSignal<Angle> _hoodAngleGetter = _hoodMotor.getPosition();

  @Logged(name = "Velocity Threshold")
  private final AngularVelocity velocityThreshold = RotationsPerSecond.of(3);

  public Shooter() {
    var frontMotorConfig = new TalonFXConfiguration();
    var frontFollowerMotorConfig = new TalonFXConfiguration();
    var backMotorConfig = new TalonFXConfiguration();
    var hoodMotorConfig = new TalonFXConfiguration();

    // front motor configs
    frontMotorConfig.CurrentLimits.StatorCurrentLimit = 100;
    frontMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    frontMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = false;

    frontMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    frontMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    frontMotorConfig.Slot0.kS = ShooterConstants.frontFlywheelkS.in(Volts);
    frontMotorConfig.Slot0.kV = ShooterConstants.frontFlywheelkV.in(Volts.per(RotationsPerSecond));

    frontMotorConfig.Slot0.kP = ShooterConstants.frontFlywheelkP.in(Volts.per(RotationsPerSecond));

    frontMotorConfig.Feedback.SensorToMechanismRatio = ShooterConstants.frontFlywheelGearRatio;

    // front follower motor configs
    frontFollowerMotorConfig.CurrentLimits.StatorCurrentLimit = 100;
    frontFollowerMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    frontFollowerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = false;

    frontFollowerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // back motor configs
    backMotorConfig.CurrentLimits.StatorCurrentLimit = 100;
    backMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    backMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = false;

    backMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    backMotorConfig.Slot0.kS = ShooterConstants.backFlywheelkS.in(Volts);
    backMotorConfig.Slot0.kV = ShooterConstants.backFlywheelkV.in(Volts.per(RotationsPerSecond));

    backMotorConfig.Slot0.kP = ShooterConstants.backFlywheelkP.in(Volts.per(RotationsPerSecond));

    backMotorConfig.Feedback.SensorToMechanismRatio = ShooterConstants.backFlywheelGearRatio;

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

    CTREUtil.attempt(() -> _frontMotor.getConfigurator().apply(frontMotorConfig), _frontMotor);
    CTREUtil.attempt(
        () -> _frontFollowerMotor.getConfigurator().apply(frontFollowerMotorConfig),
        _frontFollowerMotor);
    CTREUtil.attempt(() -> _backMotor.getConfigurator().apply(backMotorConfig), _backMotor);
    CTREUtil.attempt(() -> _hoodMotor.getConfigurator().apply(hoodMotorConfig), _hoodMotor);

    CTREUtil.attempt(() -> _frontMotor.optimizeBusUtilization(), _frontMotor);
    CTREUtil.attempt(() -> _frontFollowerMotor.optimizeBusUtilization(), _frontFollowerMotor);
    CTREUtil.attempt(() -> _backMotor.optimizeBusUtilization(), _backMotor);
    CTREUtil.attempt(() -> _hoodMotor.optimizeBusUtilization(), _hoodMotor);

    CTREUtil.attempt(
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                _frontVelocityGetter,
                _frontMotor.getSupplyCurrent(),
                _frontMotor.getStatorCurrent()),
        _frontMotor);

    CTREUtil.attempt(
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                _backVelocityGetter,
                _backMotor.getSupplyCurrent(),
                _backMotor.getStatorCurrent()),
        _backMotor);

    FaultLogger.register(_frontMotor);
    FaultLogger.register(_frontFollowerMotor);
    FaultLogger.register(_backMotor);
    FaultLogger.register(_hoodMotor);

    _frontFollowerMotor.setControl(
        new Follower(ShooterConstants.frontMotorID, MotorAlignmentValue.Opposed));
  }

  private void setFrontSpeed(AngularVelocity desiredFrontSpeed) {
    double errorRps = desiredFrontSpeed.minus(getFrontSpeed()).in(RotationsPerSecond);

    if (Math.abs(errorRps) > velocityThreshold.in(RotationsPerSecond)) {
      _frontMotor.setControl(_frontDutyCycleSetter.withOutput(Math.signum(errorRps)));
    } else {
      _frontMotor.setControl(_frontVelocitySetter.withVelocity(desiredFrontSpeed));
    }
  }

  private void setBackSpeed(AngularVelocity desiredBackSpeed) {
    double errorRps = desiredBackSpeed.minus(getBackSpeed()).in(RotationsPerSecond);

    if (Math.abs(errorRps) > velocityThreshold.in(RotationsPerSecond)) {
      _backMotor.setControl(_backDutyCycleSetter.withOutput(Math.signum(errorRps)));
    } else {
      _backMotor.setControl(_backVelocitySetter.withVelocity(desiredBackSpeed));
    }
  }

  private void setHoodAngle(Angle angle) {
    _hoodMotor.setControl(_hoodAngleSetter.withPosition(angle));
  }

  /** Score. */
  public Command score() {
    return run(() -> {
          setFrontSpeed(RotationsPerSecond.zero());
          setBackSpeed(RotationsPerSecond.zero());
          setHoodAngle(Rotations.zero());
        })
        .withName("Score");
  }

  /** Ferry. */
  public Command ferry() {
    return run(
        () -> {
          setFrontSpeed(RotationsPerSecond.zero());
          setBackSpeed(RotationsPerSecond.zero());
          setHoodAngle(Rotations.zero());
        });
  }

  /** Spits the balls in front of the robot at a fixed angle. */
  public Command spit() {
    return run(
        () -> {
          setFrontSpeed(RotationsPerSecond.zero());
          setBackSpeed(RotationsPerSecond.zero());
          setHoodAngle(Rotations.zero());
        });
  }

  @Logged(name = "Front Speed")
  public AngularVelocity getFrontSpeed() {
    return _frontVelocityGetter.refresh().getValue();
  }

  @Logged(name = "Back Speed")
  public AngularVelocity getBackSpeed() {
    return _backVelocityGetter.refresh().getValue();
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
    _frontMotor.close();
    _frontFollowerMotor.close();
    _backMotor.close();
    _hoodMotor.close();
  }
}
