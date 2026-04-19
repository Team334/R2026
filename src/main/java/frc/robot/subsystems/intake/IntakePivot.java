package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.AdvancedSubsystem;
import frc.lib.CTREUtil;
import frc.lib.FaultLogger;
import frc.lib.SysId;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Robot;
import frc.robot.utils.ShotParameters;
import java.util.function.Supplier;

public class IntakePivot extends AdvancedSubsystem {
  private final TalonFX _pivotMotor =
      new TalonFX(IntakeConstants.pivotMotorID, Constants.subsystemBus);

  private final DynamicMotionMagicVoltage _pivotAngleSetter =
      new DynamicMotionMagicVoltage(0, 0, 0);

  private final VelocityVoltage _pivotVelocitySetter = new VelocityVoltage(0);
  private final VoltageOut _pivotVoltageSetter = new VoltageOut(0);

  private final StatusSignal<Angle> _pivotAngleGetter = _pivotMotor.getPosition();

  @Logged(name = "Lower Default")
  private boolean _lowerDefault = true;

  @Logged(name = "Lower Depot")
  public boolean lowerDepot = false;

  private final Supplier<ShotParameters> _shotParametersSupplier;

  private DCMotorSim _pivotSim;

  private Notifier _simNotifier;
  private double _lastSimTime;

  private final SysIdRoutine _pivotRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(1).per(Seconds),
              Volts.of(2),
              null,
              state -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (Voltage volts) -> _pivotMotor.setControl(_pivotVoltageSetter.withOutput(volts)),
              null,
              this));

  public IntakePivot(Supplier<ShotParameters> shotParametersSupplier) {
    _shotParametersSupplier = shotParametersSupplier;

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

    pivotMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    _pivotAngleSetter
        .withVelocity(IntakeConstants.pivotVelocity)
        .withAcceleration(IntakeConstants.pivotAcceleration);

    CTREUtil.attempt(() -> _pivotMotor.getConfigurator().apply(pivotMotorConfigs), _pivotMotor);
    CTREUtil.attempt(() -> _pivotMotor.setPosition(IntakeConstants.pivotRaised), _pivotMotor);

    CTREUtil.attempt(() -> _pivotMotor.optimizeBusUtilization(), _pivotMotor);

    CTREUtil.attempt(
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                _pivotMotor.getMotorVoltage(),
                _pivotMotor.getVelocity(),
                _pivotMotor.getPosition()),
        _pivotMotor);

    FaultLogger.register(_pivotMotor);

    SysId.displayRoutine(
        "Intake Pivot",
        _pivotRoutine,
        () -> _pivotAngleGetter.refresh().getValue().gte(Rotations.of(0.49)),
        () -> _pivotAngleGetter.refresh().getValue().lte(Rotations.of(0.23)));

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

    setDefaultCommand(_lowerDefault ? lower() : raise());
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

  @Logged(name = "Angle")
  public Angle getAngle() {
    return _pivotAngleGetter.refresh().getValue();
  }

  /** Toggles the lower default, then schedules the default. */
  public Command toggleLowerDefault() {
    return runOnce(
        () -> {
          _lowerDefault = !_lowerDefault;
          setDefaultCommand(_lowerDefault ? lower() : raise());
        });
  }

  /** Drives pivot with voltage, finally resetting the angle. */
  public Command reset() {
    return run(() -> {
          _pivotMotor.setControl(_pivotVoltageSetter.withOutput(-1));
        })
        .finallyDo(() -> _pivotMotor.setPosition(IntakeConstants.pivotRaised));
  }

  /** Raises the intake. */
  public Command raise() {
    return run(() -> {
          _pivotMotor.setControl(
              _pivotAngleSetter
                  .withPosition(IntakeConstants.pivotRaised)
                  .withVelocity(IntakeConstants.pivotVelocity)
                  .withAcceleration(IntakeConstants.pivotAcceleration));
        })
        .withName("Raise");
  }

  /** Raises the intake slower while shooting */
  public Command raiseShooting() {
    return run(() -> {
          if (!_shotParametersSupplier.get().isReadyToShoot()) {
            _pivotMotor.setControl(_pivotVelocitySetter.withVelocity(0));
            return;
          }

          _pivotMotor.setControl(
              _pivotAngleSetter
                  .withPosition(IntakeConstants.pivotRaised)
                  .withVelocity(IntakeConstants.shootingPivotVelocity)
                  .withAcceleration(IntakeConstants.shootingPivotAcceleration));
        })
        .withName("Raise Shooting");
  }

  /**
   * Lowers the intake. If {@link #lowerDepot} is true, the pivot will be lowered to the depot
   * angle.
   */
  public Command lower() {
    return run(() -> {
          _pivotMotor.setControl(
              _pivotAngleSetter
                  .withPosition(
                      lowerDepot ? IntakeConstants.pivotLoweredDepot : IntakeConstants.pivotLowered)
                  .withVelocity(IntakeConstants.pivotVelocity)
                  .withAcceleration(IntakeConstants.pivotAcceleration));
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
