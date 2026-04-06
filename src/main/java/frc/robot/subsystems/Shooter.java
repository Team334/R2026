package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
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
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShotConstants;
import frc.robot.Robot;
import frc.robot.utils.ShotParameters;
import java.util.function.Supplier;

public class Shooter extends AdvancedSubsystem {
  private final TalonFX _flywheelMotor =
      new TalonFX(ShooterConstants.flywheelMotorID, Constants.subsystemBus);
  private final TalonFX _flywheelFollowerMotor =
      new TalonFX(ShooterConstants.flywheelFollowerMotorID, Constants.subsystemBus);

  private final VelocityVoltage _flywheelVelocitySetter = new VelocityVoltage(0);
  private final DutyCycleOut _flywheelDutyCycleSetter = new DutyCycleOut(0);

  private final VoltageOut _flywheelVoltageSetter = new VoltageOut(0);

  private final StatusSignal<AngularVelocity> _flywheelVelocityGetter =
      _flywheelMotor.getVelocity();

  private final StatusSignal<Double> _flywheelReferenceGetter =
      _flywheelMotor.getClosedLoopReference();
  private final MutAngularVelocity _flywheelReference = RotationsPerSecond.mutable(0);

  @Logged(name = "Idle Velocity Percentage")
  private final double idleVelocityPercentage = 0.5;

  private boolean _inTolerance = false;

  private final Supplier<ShotParameters> _shotParametersSupplier;

  private final SysIdRoutine _flywheelRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(4),
              Seconds.of(5),
              state -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (Voltage volts) ->
                  _flywheelMotor.setControl(_flywheelVoltageSetter.withOutput(volts)),
              null,
              this));

  private DCMotorSim _flywheelSim;

  private Notifier _simNotifier;
  private double _lastSimTime;

  public Shooter(Supplier<ShotParameters> shotParametersSupplier) {
    _shotParametersSupplier = shotParametersSupplier;

    var flywheelMotorConfigs = new TalonFXConfiguration();
    var flywheelFollowerMotorConfigs = new TalonFXConfiguration();

    // flywheel motor configs
    flywheelMotorConfigs.CurrentLimits.StatorCurrentLimit = 100;
    flywheelMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

    flywheelMotorConfigs.CurrentLimits.SupplyCurrentLimit = 50;
    flywheelMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

    flywheelMotorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    flywheelMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    flywheelMotorConfigs.Slot0.kS = ShooterConstants.flywheelkS.in(Volts);
    flywheelMotorConfigs.Slot0.kV = ShooterConstants.flywheelkV.in(Volts.per(RotationsPerSecond));

    flywheelMotorConfigs.Slot0.kP = ShooterConstants.flywheelkP.in(Volts.per(RotationsPerSecond));

    flywheelMotorConfigs.Feedback.SensorToMechanismRatio = ShooterConstants.flywheelGearRatio;

    // flywheel follower motor configs
    flywheelFollowerMotorConfigs.CurrentLimits.StatorCurrentLimit = 100;
    flywheelFollowerMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

    flywheelFollowerMotorConfigs.CurrentLimits.SupplyCurrentLimit = 50;
    flywheelFollowerMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

    flywheelFollowerMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    flywheelFollowerMotorConfigs.Feedback.SensorToMechanismRatio =
        ShooterConstants.flywheelGearRatio;

    CTREUtil.attempt(
        () -> _flywheelMotor.getConfigurator().apply(flywheelMotorConfigs), _flywheelMotor);
    CTREUtil.attempt(
        () -> _flywheelFollowerMotor.getConfigurator().apply(flywheelFollowerMotorConfigs),
        _flywheelFollowerMotor);

    CTREUtil.attempt(() -> _flywheelMotor.optimizeBusUtilization(), _flywheelMotor);
    CTREUtil.attempt(() -> _flywheelFollowerMotor.optimizeBusUtilization(), _flywheelFollowerMotor);

    CTREUtil.attempt(
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                200,
                _flywheelVelocityGetter,
                _flywheelMotor.getMotorVoltage(),
                _flywheelMotor.getPosition(),
                _flywheelMotor.getSupplyCurrent(),
                _flywheelMotor.getStatorCurrent()),
        _flywheelMotor);

    CTREUtil.attempt(
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                _flywheelFollowerMotor.getVelocity(),
                _flywheelFollowerMotor.getSupplyCurrent(),
                _flywheelFollowerMotor.getStatorCurrent()),
        _flywheelFollowerMotor);

    FaultLogger.register(_flywheelMotor);
    FaultLogger.register(_flywheelFollowerMotor);

    SysId.displayRoutine("Shooter Flywheel", _flywheelRoutine);

    _flywheelFollowerMotor.setControl(
        new Follower(ShooterConstants.flywheelMotorID, MotorAlignmentValue.Opposed));

    setDefaultCommand(idle());

    if (Robot.isSimulation()) {
      var flywheelMotorSimConfigs = new TalonFXConfiguration();

      _flywheelMotor.getConfigurator().refresh(flywheelMotorSimConfigs);

      flywheelMotorSimConfigs.Slot0.kS = 0;

      _flywheelMotor.getConfigurator().apply(flywheelMotorSimConfigs);

      _flywheelSim =
          new DCMotorSim(
              LinearSystemId.createDCMotorSystem(
                  MotorConstants.krakenX44, 0.001, ShooterConstants.flywheelGearRatio),
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

              _lastSimTime = currentTime;
            });

    _simNotifier.setName("Shooter Sim Thread");
    _simNotifier.startPeriodic(1 / Constants.simNotifierFrequency.in(Hertz));
  }

  private void setFlywheelSpeed(AngularVelocity speed) {
    double errorRPS = speed.minus(getFlywheelSpeed()).in(RotationsPerSecond);
    double toleranceRPS =
        _inTolerance
            ? ShooterConstants.shootingVelocityTolerance.in(RotationsPerSecond)
            : ShooterConstants.windupVelocityTolerance.in(RotationsPerSecond);

    if (Math.abs(errorRPS) > toleranceRPS) {
      _inTolerance = false;
    } else {
      _inTolerance = true;
    }

    // asymmetrical bang-bang for speeding up
    if (errorRPS > ShooterConstants.bangBangVelocityTolerance.in(RotationsPerSecond)) {
      _flywheelMotor.setControl(_flywheelDutyCycleSetter.withOutput(1));
    } else {
      _flywheelMotor.setControl(_flywheelVelocitySetter.withVelocity(speed));
    }
  }

  /** Set flywheel to {@link #idleVelocityPercentage} of shooting speed. */
  public Command idle() {
    return run(() -> {
          ShotParameters parameters = _shotParametersSupplier.get();

          setFlywheelSpeed(parameters.getFlywheelSpeed().times(idleVelocityPercentage));
        })
        .withName("Idle");
  }

  /** Scores / ferries depending on robot pose. */
  public Command shoot() {
    return run(() -> {
          ShotParameters parameters = _shotParametersSupplier.get();

          setFlywheelSpeed(parameters.getFlywheelSpeed());
        })
        .withName("Shoot");
  }

  /** Spits the fuel in front of the robot at a fixed angle and speed. */
  public Command spit() {
    return run(() -> {
          setFlywheelSpeed(ShotConstants.spitFlywheelSpeed);
        })
        .withName("Spit");
  }

  /** Whether the flywheel velocity is in tolerance of its reference. */
  @Logged(name = "In Tolerance")
  public boolean inTolerance() {
    return _inTolerance;
  }

  @Logged(name = "Flywheel Speed")
  public AngularVelocity getFlywheelSpeed() {
    return _flywheelVelocityGetter.refresh().getValue();
  }

  @Logged(name = "Flywheel Reference")
  public AngularVelocity getFlywheelReference() {
    return _flywheelReference.mut_setMagnitude(_flywheelReferenceGetter.refresh().getValue());
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
  }
}
