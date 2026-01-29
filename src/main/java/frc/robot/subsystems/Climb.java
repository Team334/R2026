// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
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
import frc.robot.Constants.ClimbConstants;
import frc.robot.Robot;

public class Climb extends AdvancedSubsystem {
  private final TalonFX _climbMotor =
      new TalonFX(ClimbConstants.climbMotorID, Constants.subsystemBus);

  private final MotionMagicVoltage _heightSetter = new MotionMagicVoltage(0);

  private final StatusSignal<Angle> _heightGetter = _climbMotor.getPosition();

  private final Mechanism2d _mech = new Mechanism2d(1.85, 1);
  private final MechanismRoot2d _root = _mech.getRoot("climb", 1, 0.1);
  private final MechanismLigament2d _climbElevator =
      _root.append(new MechanismLigament2d("climb", 0.3, 90, 3, new Color8Bit(Color.kAqua)));

  private ElevatorSim _climbSim;
  private double _lastSimTime;
  private Notifier _simNotifier;

  public Climb() {
    var climbMotorConfigs = new TalonFXConfiguration();
    var climbSlot = new SlotConfigs();

    climbSlot.kS = ClimbConstants.climbkS.in(Volts);
    climbSlot.kG = ClimbConstants.climbkG.in(Volts);
    climbSlot.kV = ClimbConstants.climbkV.in(Volts.per(RotationsPerSecond));
    climbSlot.kA = ClimbConstants.climbkA.in(Volts.per(RotationsPerSecondPerSecond));

    climbSlot.kP = ClimbConstants.climbkP.in(Volts.per(Rotations));

    climbMotorConfigs.Slot0 = Slot0Configs.from(climbSlot);

    climbMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    climbMotorConfigs.Feedback.SensorToMechanismRatio = ClimbConstants.climbGearRatio;

    climbMotorConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        ClimbConstants.maxElevatorHeight.in(Rotations);
    climbMotorConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        ClimbConstants.minElevatorHeight.in(Rotations);

    climbMotorConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    climbMotorConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    CTREUtil.attempt(() -> _climbMotor.getConfigurator().apply(climbMotorConfigs), _climbMotor);
    CTREUtil.attempt(() -> _climbMotor.optimizeBusUtilization(), _climbMotor);

    FaultLogger.register(_climbMotor);

    if (Robot.isSimulation()) {
      _climbMotor.setPosition(0);

      _climbSim =
          new ElevatorSim(
              DCMotor.getKrakenX60Foc(1),
              Constants.ClimbConstants.climbGearRatio,
              Units.lbsToKilograms(6),
              Constants.ClimbConstants.drumRadius.in(Meters),
              Constants.ClimbConstants.minElevatorHeight.in(Rotations)
                  * Constants.ClimbConstants.drumCircumference.in(Meters),
              Constants.ClimbConstants.maxElevatorHeight.in(Rotations)
                  * Constants.ClimbConstants.drumCircumference.in(Meters),
              true,
              // initial height
              Constants.ClimbConstants.minElevatorHeight.in(Rotations)
                  * Constants.ClimbConstants.drumCircumference.in(Meters));

      SmartDashboard.putData("Climb Visualizer", _mech);
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

              var climbMotorState = _climbMotor.getSimState();
              climbMotorState.setSupplyVoltage(batteryVolts);

              _climbSim.setInputVoltage(climbMotorState.getMotorVoltageMeasure().in(Volts));
              _climbSim.update(deltaTime);

              climbMotorState.setRawRotorPosition(
                  _climbSim.getPositionMeters()
                      / ClimbConstants.drumCircumference.in(Meters)
                      * ClimbConstants.climbGearRatio);

              climbMotorState.setRotorVelocity(
                  _climbSim.getPositionMeters()
                      / ClimbConstants.drumCircumference.in(Meters)
                      * ClimbConstants.climbGearRatio);

              _lastSimTime = currentTime;
            });
    _simNotifier.setName("Climb Sim Thread");
    _simNotifier.startPeriodic(1 / Constants.simNotifierFrequency.in(Hertz));
  }

  @Override
  public void periodic() {
    DogLog.time("Time/Wristevator/periodic()");
    super.periodic();
  }

  @Override
  public void simulationPeriodic() {
    _climbElevator.setLength(
        Units.radiansToRotations(
            getHeight()
                - ClimbConstants.maxElevatorHeight.in(Radians)
                    * ClimbConstants.drumCircumference.in(Meters)));
  }

  private void setHeight(Angle height) {
    _climbMotor.setControl(_heightSetter.withPosition(height.in(Rotations)));
  }

  public Command extend() {
    return run(() -> setHeight(ClimbConstants.maxElevatorHeight)).withName("Extend Climb");
  }

  public Command retract() {
    return run(() -> setHeight(ClimbConstants.minElevatorHeight)).withName("Retract Climb");
  }

  public Command goToHeight(Angle target) {
    return run(() -> setHeight(target)).withName("Climb to Position");
  }

  @Logged(name = "Climb Height")
  public double getHeight() {
    return _heightGetter.refresh().getValue().in(Radians);
  }

  @Override
  public void close() {
    _climbMotor.close();
    _simNotifier.close();
  }
}
