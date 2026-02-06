package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.AdvancedSubsystem;
import frc.lib.CTREUtil;
import frc.lib.FaultLogger;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Robot;

public class Intake extends AdvancedSubsystem {
  private final TalonFX _feedMotor =
      new TalonFX(IntakeConstants.feedMotorID, Constants.subsystemBus);

  private final TalonFX _pivotMotor =
      new TalonFX(IntakeConstants.pivotMotorID, Constants.subsystemBus);

  private final MotionMagicVoltage _pivotAngleSetter = new MotionMagicVoltage(0);
  private final VelocityVoltage _feedVelocitySetter = new VelocityVoltage(0);

  // private final VoltageOut _pivotVoltageSetter = new VoltageOut(0);
  // private final VoltageOut _feedVoltageSetter = new VoltageOut(0);

  private final StatusSignal<Angle> _pivotAngleGetter = _pivotMotor.getPosition();
  private final StatusSignal<AngularVelocity> _feedVelocityGetter = _feedMotor.getVelocity();

  @Logged(name = "Intake Lowered")
  private boolean _intakeLowered = false;

  private final Mechanism2d _mech = new Mechanism2d(1.85, 1);
  private final MechanismRoot2d _root = _mech.getRoot("intake", 0.5, 0.1);
  private final MechanismLigament2d _intake =
      _root.append(new MechanismLigament2d("intake", 0.5, 0, 3, new Color8Bit(Color.kBlue)));

  private SingleJointedArmSim _pivotSim;

  private Notifier _simNotifier;
  private double _lastSimTime;

  public Intake() {
    var feedMotorConfigs = new TalonFXConfiguration();
    var pivotMotorConfigs = new TalonFXConfiguration();

    // feed motor configs
    feedMotorConfigs.Slot0.kS = IntakeConstants.feedkS.in(Volts);
    feedMotorConfigs.Slot0.kV = IntakeConstants.feedkV.in(Volts.per(RotationsPerSecond));

    feedMotorConfigs.Slot0.kP = IntakeConstants.feedkP.in(Volts.per(RotationsPerSecond));

    feedMotorConfigs.Feedback.SensorToMechanismRatio = IntakeConstants.feedGearRatio;

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

    FaultLogger.register(_feedMotor);
    FaultLogger.register(_pivotMotor);

    setDefaultCommand(raise());

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
  public Angle getAngle() {
    return _pivotAngleGetter.refresh().getValue();
  }

  @Logged(name = "Speed")
  public AngularVelocity getSpeed() {
    return _feedVelocityGetter.refresh().getValue();
  }

  /** Raises the intake. Command runs forever. */
  public Command raise() {
    return run(() -> {
          _pivotMotor.setControl(_pivotAngleSetter.withPosition(0));
        })
        .beforeStarting(runOnce(() -> _intakeLowered = false))
        .withName("Raise");
  }

  /** Lowers the intake. Command ends once intake is lowered. */
  public Command lower() {
    return run(() -> {
          _pivotMotor.setControl(_pivotAngleSetter.withPosition(0));
        })
        .until(() -> true)
        .andThen(runOnce(() -> _intakeLowered = true))
        .withName("Lower");
  }

  /** Runs the feed wheels inwards, waiting for the intake to be lowered first. */
  public Command feedIn() {
    return Commands.runOnce(() -> _feedMotor.setControl(_feedVelocitySetter.withVelocity(0)))
        .onlyIf(() -> _intakeLowered);
  }

  /** Runs the feed wheels outwards, waiting for the intake to be lowered first. */
  public Command feedOut() {
    return Commands.runOnce(() -> _feedMotor.setControl(_feedVelocitySetter.withVelocity(0)))
        .onlyIf(() -> _intakeLowered);
  }

  /** Stops the feed wheels, waiting for the intake to be lowered first. */
  public Command feedStop() {
    return Commands.runOnce(() -> _feedMotor.setControl(_feedVelocitySetter.withVelocity(0)))
        .onlyIf(() -> _intakeLowered);
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

    _intake.setAngle(getAngle().in(Degrees));
  }

  @Override
  public void close() {
    _pivotMotor.close();
    _feedMotor.close();

    _simNotifier.close();
  }
}
