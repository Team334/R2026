package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
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
import java.util.function.BooleanSupplier;

public class IntakePivot extends AdvancedSubsystem {
  private final TalonFX _pivotMotor =
      new TalonFX(IntakeConstants.pivotMotorID, Constants.subsystemBus);

  private final MotionMagicVoltage _pivotAngleSetter = new MotionMagicVoltage(0);
  private final StatusSignal<Angle> _pivotAngleGetter = _pivotMotor.getPosition();

  private final Trigger _intakeLowered =
      new Trigger(
              () ->
                  MathUtil.isNear(
                      IntakeConstants.pivotTucked.in(Degrees), getAngle().in(Degrees), 3))
          .debounce(0.5);

  private final BooleanSupplier _inBumpZoneSupplier;

  private DCMotorSim _pivotSim;

  private Notifier _simNotifier;
  private double _lastSimTime;

  public IntakePivot(BooleanSupplier inBumpZoneSupplier) {
    _inBumpZoneSupplier = inBumpZoneSupplier;

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
        IntakeConstants.pivotForwardSoftLimitThreshold.in(Rotations);
    pivotMotorConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        IntakeConstants.pivotReverseSoftLimitThreshold.in(Rotations);

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

    if (Robot.isSimulation()) {
      // rely on sim to control the position
      _pivotMotor.setPosition(0);

      // prevent setRawMotor_ from negating physics sim output
      var pivotMotorSimConfigs = new TalonFXConfiguration();

      _pivotMotor.getConfigurator().refresh(pivotMotorSimConfigs);

      pivotMotorSimConfigs.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);

      pivotMotorSimConfigs.Slot0.kS = 0;
      pivotMotorSimConfigs.Slot0.kG = 0;

      _pivotMotor.getConfigurator().apply(pivotMotorSimConfigs);

      _pivotSim =
          new DCMotorSim(
              LinearSystemId.createDCMotorSystem(
                  IntakeConstants.pivotkV.in(Volts.per(RadiansPerSecond)),
                  IntakeConstants.pivotkA.in(Volts.per(RadiansPerSecondPerSecond))),
              MotorConstants.krakenX44);

      _pivotSim.setAngle(IntakeConstants.pivotRaised.in(Radians));

      _pivotAngleSetter.withUpdateFreqHz(Hertz.of(1000));
      _pivotAngleGetter.setUpdateFrequency(Hertz.of(1000));

      startSimThread();
    }

    setDefaultCommand(raise());
  }

  private void startSimThread() {
    _lastSimTime = Utils.getCurrentTimeSeconds();

    _simNotifier =
        new Notifier(
            () -> {
              final double batteryVolts = RobotController.getBatteryVoltage();

              final double currentTime = Utils.getCurrentTimeSeconds();
              final double deltaTime = currentTime - _lastSimTime;

              var pivotMotorSimState = _pivotMotor.getSimState();

              pivotMotorSimState.setSupplyVoltage(batteryVolts);

              _pivotSim.setInputVoltage(pivotMotorSimState.getMotorVoltageMeasure().in(Volts));
              _pivotSim.update(deltaTime);

              pivotMotorSimState.setRawRotorPosition(
                  _pivotSim.getAngularPosition().times(IntakeConstants.pivotGearRatio));
              pivotMotorSimState.setRotorVelocity(
                  _pivotSim.getAngularVelocity().times(IntakeConstants.pivotGearRatio));

              _lastSimTime = currentTime;
            });

    _simNotifier.setName("IntakePivot Sim Thread");
    _simNotifier.startPeriodic(1 / Constants.simNotifierFrequency.in(Hertz));
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

  /** Lowers the intake, tucking it if necessary. */
  public Command lower() {
    return run(() -> {
          if (_inBumpZoneSupplier.getAsBoolean()) {
            _pivotMotor.setControl(_pivotAngleSetter.withPosition(IntakeConstants.pivotTucked));
            return;
          }

          _pivotMotor.setControl(_pivotAngleSetter.withPosition(IntakeConstants.pivotLowered));
        })
        .withName("Lower");
  }

  @Override
  public void periodic() {
    DogLog.time("Timing/IntakePivot/periodic()");
    super.periodic();
    DogLog.timeEnd("Timing/IntakePivot/periodic()");
  }

  @Override
  public void close() {
    _pivotMotor.close();
  }
}
