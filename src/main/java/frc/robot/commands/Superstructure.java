// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.InputStream;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.intake.IntakeFeed;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.utils.FieldUtil;
import frc.robot.utils.ShotParameters;
import java.util.function.Supplier;

/** All superstructure commands. */
public class Superstructure {
  private final Shooter _shooter;
  private final Hopper _hopper;
  private final IntakePivot _intakePivot;
  private final Climb _climb;
  private final Swerve _swerve;

  private final Supplier<ShotParameters> _shotParametersSupplier;

  public Superstructure(
      Shooter shooter,
      Hopper hopper,
      IntakePivot intakePivot,
      IntakeFeed intakeFeed,
      Climb climb,
      Swerve swerve,
      Supplier<ShotParameters> shotParametersSupplier) {
    _shooter = shooter;
    _hopper = hopper;
    _intakePivot = intakePivot;
    _climb = climb;
    _swerve = swerve;

    _shotParametersSupplier = shotParametersSupplier;
  }

  /** Scores / ferries depending on robot pose. */
  public Command shoot(InputStream velX, InputStream velY) {
    return parallel(
            _shooter.shoot(),
            _hopper.feedShot(),
            _intakePivot.raiseShooting(),
            _swerve.driveFacing(velX, velY, () -> _shotParametersSupplier.get().getShotHeading()))
        .withName("Shoot");
  }

  /** Shoots using manually set values and driver control. */
  public Command shootManually(InputStream velX, InputStream velY, InputStream velOmega) {
    return parallel(
            _shooter.shoot(),
            _hopper.feedShot(),
            _intakePivot.raiseShooting(),
            _swerve.drive(velX, velY, velOmega))
        .beforeStarting(
            () -> {
              _shotParametersSupplier.get().isManual = true;
              _swerve.isOpenLoop = true;
            })
        .finallyDo(() -> _shotParametersSupplier.get().isManual = false)
        .withName("Shoot Manually");
  }

  /** Spits fuel at a short range without aiming. */
  public Command spit() {
    return parallel(_shooter.spit(), _hopper.feedSpit()).withName("Spit");
  }

  /**
   * Drives while extending the climb, stops before the tower to let the climb extend, and finally
   * drives to L1 before climbing.
   */
  public Command climbRoutine() {
    return sequence(
            // drive to pre-climb pose while extending, wait for climb to finish extending,
            // and finally drive to climb pose and climb
            parallel(
                _swerve.driveTo(FieldUtil::getPreClimb),
                _climb
                    .extend()
                    .until(
                        () ->
                            _climb
                                .getHeight()
                                .isNear(
                                    ClimbConstants.extended, ClimbConstants.extendedTolerance))),
            _swerve.driveTo(FieldUtil::getClimb),
            _climb.climb())
        .withName("Climb Routine");
  }
}
