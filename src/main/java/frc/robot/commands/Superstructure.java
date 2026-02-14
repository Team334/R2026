// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.InputStream;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.intake.IntakeFeed;
import frc.robot.subsystems.intake.IntakePivot;
import java.util.function.Supplier;

/** All superstructure commands. */
public class Superstructure {
  private final Shooter _shooter;
  private final Hopper _hopper;
  private final IntakePivot _intakePivot;
  private final IntakeFeed _intakeFeed;
  private final Climb _climb;
  private final Swerve _swerve;

  private final Supplier<Rotation2d> _shotHeadingSupplier;

  public Superstructure(
      Shooter shooter,
      Hopper hopper,
      IntakePivot intakePivot,
      IntakeFeed intakeFeed,
      Climb climb,
      Swerve swerve,
      Supplier<Rotation2d> shotHeadingSupplier) {
    _shooter = shooter;
    _hopper = hopper;
    _intakePivot = intakePivot;
    _intakeFeed = intakeFeed;
    _climb = climb;
    _swerve = swerve;

    _shotHeadingSupplier = shotHeadingSupplier;
  }

  /** Scores / ferries depending on robot pose. */
  public Command shoot(InputStream velX, InputStream velY) {
    return parallel(
            _shooter.shoot(),
            _hopper.feedShot(),
            _swerve.driveFacing(velX, velY, () -> _shotHeadingSupplier.get()))
        .withName("Shoot");
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
    return run(() -> {}).withName("Climb Routine");
  }
}
