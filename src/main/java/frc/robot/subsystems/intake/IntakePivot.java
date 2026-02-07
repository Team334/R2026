package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.AdvancedSubsystem;
import frc.lib.CTREUtil;
import frc.lib.FaultLogger;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakePivot extends AdvancedSubsystem {
  private final TalonFX _pivotMotor =
      new TalonFX(IntakeConstants.pivotMotorID, Constants.subsystemBus);

  private final MotionMagicVoltage _pivotAngleSetter = new MotionMagicVoltage(0);
  private final StatusSignal<Angle> _pivotAngleGetter = _pivotMotor.getPosition();

  private Trigger _intakeLowered =
      new Trigger(
              () ->
                  MathUtil.isNear(
                      IntakeConstants.pivotLowered.in(Degrees), getAngle().in(Degrees), 2))
          .debounce(0.5);

  public IntakePivot() {
    var pivotMotorConfigs = new TalonFXConfiguration();

    // pivot motor configs
    pivotMotorConfigs.Slot0.kS = IntakeConstants.pivotkS.in(Volts);
    pivotMotorConfigs.Slot0.kG = IntakeConstants.pivotkG.in(Volts);
    pivotMotorConfigs.Slot0.kV = IntakeConstants.pivotkV.in(Volts.per(RotationsPerSecond));
    pivotMotorConfigs.Slot0.kA = IntakeConstants.pivotkA.in(Volts.per(RotationsPerSecondPerSecond));

    pivotMotorConfigs.Slot0.kP = IntakeConstants.pivotkP.in(Volts.per(Rotations));

    pivotMotorConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    pivotMotorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    pivotMotorConfigs.Feedback.SensorToMechanismRatio = IntakeConstants.pivotGearRatio;

    pivotMotorConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        IntakeConstants.pivotLowered.in(Rotations);
    pivotMotorConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        IntakeConstants.pivotRaised.in(Rotations);

    pivotMotorConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pivotMotorConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    pivotMotorConfigs.MotionMagic.MotionMagicCruiseVelocity =
        IntakeConstants.pivotVelocity.in(RotationsPerSecond);
    pivotMotorConfigs.MotionMagic.MotionMagicAcceleration =
        IntakeConstants.pivotAcceleration.in(RotationsPerSecondPerSecond);

    pivotMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    CTREUtil.attempt(() -> _pivotMotor.getConfigurator().apply(pivotMotorConfigs), _pivotMotor);
    CTREUtil.attempt(() -> _pivotMotor.setPosition(IntakeConstants.pivotRaised), _pivotMotor);

    CTREUtil.attempt(() -> _pivotMotor.optimizeBusUtilization(), _pivotMotor);

    FaultLogger.register(_pivotMotor);

    setDefaultCommand(raise());
  }

  /** Intake is lowered trigger. */
  @Logged(name = "Intake Lowered")
  public Trigger intakeLowered() {
    return _intakeLowered;
  }

  @Logged(name = "Angle")
  public Angle getAngle() {
    return _pivotAngleGetter.refresh().getValue();
  }

  /** Raises the intake. */
  public Command raise() {
    return run(() -> {
          _pivotMotor.setControl(_pivotAngleSetter.withPosition(IntakeConstants.pivotRaised));
        })
        .withName("Raise");
  }

  /** Partially lowers intake to clear bump and protect intake with the expandable hopper. */
  public Command tuck() {
    return run(() -> {
          _pivotMotor.setControl(_pivotAngleSetter.withPosition(IntakeConstants.pivotTucked));
        })
        .withName("Tuck");
  }

  /** Lowers the intake. */
  public Command lower() {
    return run(() -> {
          _pivotMotor.setControl(_pivotAngleSetter.withPosition(IntakeConstants.pivotLowered));
        })
        .withName("Lower");
  }

  @Override
  public void periodic() {
    DogLog.time("Time/IntakePivot/periodic()");
    super.periodic();
    DogLog.timeEnd("Time/IntakePivot/periodic()");
  }

  @Override
  public void close() {
    _pivotMotor.close();
  }
}
