// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.InputStream;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.intake.IntakeFeed;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.utils.ShotParameters;
import java.util.function.Supplier;

/** All superstructure commands. */
public class Superstructure {
  private final Shooter _shooter;
  private final Hopper _hopper;
  private final IntakePivot _intakePivot;
  private final Swerve _swerve;

  private final Supplier<ShotParameters> _shotParametersSupplier;

  public Superstructure(
      Shooter shooter,
      Hopper hopper,
      IntakePivot intakePivot,
      IntakeFeed intakeFeed,
      Swerve swerve,
      Supplier<ShotParameters> shotParametersSupplier) {
    _shooter = shooter;
    _hopper = hopper;
    _intakePivot = intakePivot;
    _swerve = swerve;

    _shotParametersSupplier = shotParametersSupplier;
  }

  /** Shoots. */
  public Command shoot(InputStream velX, InputStream velY, boolean lowerPivot) {
    return parallel(
            _shooter.shoot(),
            _hopper.feedShot(),
            lowerPivot ? _intakePivot.lower() : _intakePivot.pivotShooting(),
            _swerve.driveFacing(
                velX,
                velY,
                () -> Rotation2d.fromRadians(_shotParametersSupplier.get().getShotHeading())))
        .withName("Shoot");
  }

  /**
   * Shoot to use when {@link ShotParameters#isManual} is true. Gives the driver control over omega.
   */
  public Command shootManually(
      InputStream velX, InputStream velY, InputStream velOmega, boolean lowerPivot) {
    return parallel(
            _shooter.shoot(),
            _hopper.feedShot(),
            lowerPivot ? _intakePivot.lower() : _intakePivot.pivotShooting(),
            _swerve.drive(velX, velY, velOmega))
        .beforeStarting(
            () -> {
              _swerve.isOpenLoop = true;
            })
        .withName("Shoot Manually");
  }

  /** Unjam the shooter and hopper. */
  public Command unjam() {
    return parallel(_shooter.unjam(), _hopper.unjam()).withName("Unjam");
  }
}
