package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AdvancedSubsystem;
import frc.lib.CTREUtil;
import frc.lib.FaultLogger;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends AdvancedSubsystem {
  private final TalonFX _frontMotor =
      new TalonFX(ShooterConstants.frontMotorID, Constants.canivore);
  private final TalonFX _backMotor = new TalonFX(ShooterConstants.backMotorID, Constants.canivore);

  private final VelocityVoltage _frontVelocitySetter = new VelocityVoltage(0);
  private final VelocityVoltage _backVelocitySetter = new VelocityVoltage(0);

  private final StatusSignal<AngularVelocity> _frontVelocityGetter = _frontMotor.getVelocity();
  private final StatusSignal<AngularVelocity> _backVelocityGetter = _backMotor.getVelocity();

  public Shooter() {
    var frontMotorConfig = new TalonFXConfiguration();
    var backMotorConfig = new TalonFXConfiguration();

    backMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    frontMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    frontMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    CTREUtil.attempt(() -> _frontMotor.getConfigurator().apply(frontMotorConfig), _frontMotor);
    CTREUtil.attempt(() -> _backMotor.getConfigurator().apply(backMotorConfig), _backMotor);

    CTREUtil.attempt(() -> _frontMotor.optimizeBusUtilization(), _frontMotor);
    CTREUtil.attempt(() -> _backMotor.optimizeBusUtilization(), _backMotor);

    CTREUtil.attempt(
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                _frontMotor.getPosition(),
                _frontMotor.getVelocity(),
                _frontMotor.getMotorVoltage()),
        _frontMotor);

    CTREUtil.attempt(
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                _backMotor.getPosition(),
                _backMotor.getVelocity(),
                _backMotor.getMotorVoltage()),
        _backMotor);

    FaultLogger.register(_frontMotor);
    FaultLogger.register(_backMotor);

    setDefaultCommand(setSpeed(0, 0));
  }

  private Command setSpeed(double frontSpeed, double backSpeed) {
    return run(
        () -> {
          _frontVelocitySetter.Velocity = Units.radiansToRotations(frontSpeed);
          _backVelocitySetter.Velocity = Units.radiansToRotations(backSpeed);

          _frontMotor.setControl(_frontVelocitySetter);
          _backMotor.setControl(_backVelocitySetter);
        });
  }

  public Command shoot() {
    return setSpeed(
        ShooterConstants.frontSpeed.in(RadiansPerSecond),
        ShooterConstants.backSpeed.in(RadiansPerSecond));
  }

  @Logged(name = "Front Speed")
  public double getFrontSpeed() {
    return _frontVelocityGetter.refresh().getValue().in(RadiansPerSecond);
  }

  @Logged(name = "Back Speed")
  public double getBackSpeed() {
    return _backVelocityGetter.refresh().getValue().in(RadiansPerSecond);
  }

  @Override
  public void close() {
    _frontMotor.close();
    _backMotor.close();
  }
}
