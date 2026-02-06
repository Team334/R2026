// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.intake.IntakeFeed;
import frc.robot.subsystems.intake.IntakePivot;

/** All superstructure commands. */
public class Superstructure {
  public Superstructure(
      Shooter shooter,
      Hopper hopper,
      IntakePivot intakePivot,
      IntakeFeed intakeFeed,
      Climb climb,
      Swerve swerve) {
    //
  }

  /** Scores / ferries depending on robot position. */
  public Command shoot() {
    return run(null);
  }

  /** Spits fuel at a short range without aiming. */
  public Command spit() {
    return run(null);
  }

  /**
   * Drives while extending the climb, stops before the tower to let the climb extend, and finally
   * drives to L1 before climbing.
   */
  public Command climbRoutine() {
    return run(null);
  }
}
