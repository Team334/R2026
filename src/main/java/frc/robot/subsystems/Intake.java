package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AdvancedSubsystem;
import frc.lib.CTREUtil;
import frc.lib.FaultLogger;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Robot;

public class Intake extends AdvancedSubsystem {
  private final TalonFX _feedMotor = new TalonFX(IntakeConstants.feedMotorID, "canivore");

  private final TalonFX _pivotMotor = new TalonFX(IntakeConstants.pivotMotorID, "canivore");

  private final MotionMagicVoltage _pivotSetter = new MotionMagicVoltage(0);
  private final VelocityVoltage _feedSetter = new VelocityVoltage(0);

  private final VoltageOut _pivotVoltageSetter = new VoltageOut(0);
  private final VoltageOut _feedVoltageSetter = new VoltageOut(0);

  private final StatusSignal<Angle> _pivotAngleGetter = _pivotMotor.getPosition();
  private final StatusSignal<AngularVelocity> _feedVelocityGetter = _feedMotor.getVelocity();

  private final Mechanism2d _mech = new Mechanism2d(1.85, 1);
  private final MechanismRoot2d _root = _mech.getRoot("intake", 0.5, 0.1);
  private final MechanismLigament2d _intake =
      _root.append(new MechanismLigament2d("intake", 0.5, 0, 3, new Color8Bit(Color.kBlue)));
  private SingleJointedArmSim _pivotSim;
  private Notifier _simNotifier;
  private double _lastSimTime;

  public Intake() {
    setDefaultCommand(stow());

    var feedMotorConfigs = new TalonFXConfiguration();
    var pivotMotorConfigs = new TalonFXConfiguration();

    // feed configs
    feedMotorConfigs.Slot0.kS = IntakeConstants.feedkS.in(Volts);
    feedMotorConfigs.Slot0.kV = IntakeConstants.feedkV.in(Volts.per(RotationsPerSecond));

    feedMotorConfigs.Slot0.kP = IntakeConstants.feedkP.in(Volts.per(RotationsPerSecond));

    feedMotorConfigs.Feedback.SensorToMechanismRatio = IntakeConstants.feedGearRatio;

    // pivot configs
    pivotMotorConfigs.Slot0.kS = IntakeConstants.pivotkS.in(Volts);
    pivotMotorConfigs.Slot0.kG = IntakeConstants.pivotkG.in(Volts);
    pivotMotorConfigs.Slot0.kV = IntakeConstants.pivotkV.in(Volts.per(RotationsPerSecond));
    pivotMotorConfigs.Slot0.kA = IntakeConstants.pivotkA.in(Volts.per(RotationsPerSecondPerSecond));

    pivotMotorConfigs.Slot0.kP = IntakeConstants.pivotkP.in(Volts.per(Rotations));

    pivotMotorConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    pivotMotorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    pivotMotorConfigs.Feedback.SensorToMechanismRatio = IntakeConstants.pivotGearRatio;

    pivotMotorConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        IntakeConstants.pivotOut.in(Rotations);
    pivotMotorConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        IntakeConstants.pivotStowed.in(Rotations);

    pivotMotorConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pivotMotorConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    pivotMotorConfigs.MotionMagic.MotionMagicCruiseVelocity =
        IntakeConstants.pivotVelocity.in(RotationsPerSecond);
    pivotMotorConfigs.MotionMagic.MotionMagicAcceleration =
        IntakeConstants.pivotAcceleration.in(RotationsPerSecondPerSecond);

    pivotMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    CTREUtil.attempt(() -> _feedMotor.getConfigurator().apply(feedMotorConfigs), _feedMotor);

    CTREUtil.attempt(() -> _pivotMotor.getConfigurator().apply(pivotMotorConfigs), _pivotMotor);
    CTREUtil.attempt(() -> _pivotMotor.setPosition(IntakeConstants.pivotStowed), _pivotMotor);

    CTREUtil.attempt(() -> _feedMotor.optimizeBusUtilization(), _feedMotor);
    CTREUtil.attempt(() -> _pivotMotor.optimizeBusUtilization(), _pivotMotor);

    CTREUtil.attempt(
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                _feedMotor.getPosition(),
                _feedMotor.getVelocity(),
                _feedMotor.getMotorVoltage()),
        _feedMotor);

    CTREUtil.attempt(
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                _pivotMotor.getPosition(),
                _pivotMotor.getVelocity(),
                _pivotMotor.getMotorVoltage()),
        _pivotMotor);

    FaultLogger.register(_feedMotor);
    FaultLogger.register(_pivotMotor);

    // sim
    if (Robot.isSimulation()) {
      _pivotMotor.setPosition(0);

      var config = new MotorOutputConfigs();

      _pivotMotor.getConfigurator().refresh(config);
      _pivotMotor
          .getConfigurator()
          .apply(config.withInverted(InvertedValue.CounterClockwise_Positive));

      SmartDashboard.putData("Intake Visualizer", _mech);

      _pivotSim =
          new SingleJointedArmSim(
              DCMotor.getKrakenX60(1),
              IntakeConstants.pivotGearRatio,
              SingleJointedArmSim.estimateMOI(
                  IntakeConstants.intakeLength.in(Meters), Units.lbsToKilograms(12)),
              IntakeConstants.intakeLength.in(Meters),
              IntakeConstants.pivotStowed.in(Radians),
              IntakeConstants.pivotOut.in(Radians),
              true,
              IntakeConstants.pivotStowed.in(Radians));

      startSimThread();
    }
  }

  public void startSimThread() {
    _lastSimTime = Utils.getCurrentTimeSeconds();

    _simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              final double deltaTime = currentTime - _lastSimTime;

              final double batteryVoltage = RobotController.getBatteryVoltage();

              var pivotMotorSimState = _pivotMotor.getSimState();

              pivotMotorSimState.setSupplyVoltage(batteryVoltage);

              _pivotSim.setInputVoltage(pivotMotorSimState.getMotorVoltageMeasure().in(Volts));

              _pivotSim.update(deltaTime);

              pivotMotorSimState.setRawRotorPosition(
                  Units.radiansToRotations(
                      _pivotSim.getAngleRads() * IntakeConstants.pivotGearRatio));

              pivotMotorSimState.setRotorVelocity(
                  Units.radiansToRotations(
                      _pivotSim.getVelocityRadPerSec() * IntakeConstants.pivotGearRatio));

              _lastSimTime = currentTime;
            });

    _simNotifier.setName("Intake Sim Thread");
    _simNotifier.startPeriodic(1 / Constants.simNotifierFrequency.in(Hertz));
  }

  @Logged(name = "Angle")
  public double getAngle() {
    return _pivotAngleGetter.refresh().getValue().in(Radians);
  }

  @Logged(name = "Speed")
  public double getSpeed() {
    return _feedVelocityGetter.refresh().getValue().in(RadiansPerSecond);
  }

  private Command set(double pivotAngle, double feedSpeed) {
    return run(
        () -> {
          _pivotMotor.setControl(_pivotSetter.withPosition(Units.radiansToRotations(pivotAngle)));
          _feedMotor.setControl(_feedSetter.withVelocity(Units.radiansToRotations(feedSpeed)));
        });
  }

  public Command stow() {
    return set(IntakeConstants.pivotStowed.in(Radians), 0).withName("Stow");
  }

  public Command intake() {
    return set(IntakeConstants.pivotOut.in(Radians), IntakeConstants.feedSpeed.in(RadiansPerSecond))
        .withName("Intake");
  }

  /** Outtake onto the ground. */
  public Command outtake() {
    return set(
            IntakeConstants.pivotOut.in(Radians),
            IntakeConstants.feedSpeed.unaryMinus().in(RadiansPerSecond))
        .withName("Outtake");
  }

  private void setPivotVoltage(double volts) {
    _pivotMotor.setControl(_pivotVoltageSetter.withOutput(volts));
  }

  private void setFeedVoltage(double volts) {
    _feedMotor.setControl(_feedVoltageSetter.withOutput(volts));
  }

  @Override
  public void periodic() {
    DogLog.time("Time/Intake/periodic()");

    super.periodic();

    DogLog.timeEnd("Time/Intake/periodic()");
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();

    _intake.setAngle(Math.toDegrees(getAngle()));
  }

  @Override
  public void close() {
    _pivotMotor.close();
    _feedMotor.close();

    _simNotifier.close();
  }
}
