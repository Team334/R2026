// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.lib.UnitTestingUtil.*;
import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.IntakeFeed;
import frc.robot.subsystems.intake.IntakePivot;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class IntakeTest {
  private IntakePivot _intakePivot;
  private IntakeFeed _intakeFeed;

  @BeforeEach
  public void setup() {
    setupTests();

    _intakePivot = new IntakePivot();
    _intakeFeed = new IntakeFeed(_intakePivot.intakeLowered());
  }

  @AfterEach
  public void close() {
    reset(_intakePivot, _intakeFeed);
  }

  @Test
  public void feedCorrectly() {
    run(_intakePivot.raise());
    fastForward(Seconds.of(5));

    // try to feed with intake raised
    run(_intakeFeed.feedIn());
    assert _intakeFeed.getSpeed().equals(RotationsPerSecond.zero());

    run(_intakeFeed.feedOut());
    assert _intakeFeed.getSpeed().equals(RotationsPerSecond.zero());

    // try to feed with intake lowered
    run(_intakePivot.lower());
    fastForward(Seconds.of(5));

    run(_intakeFeed.feedIn());
    fastForward(Seconds.of(6));
    assertEquals(
        IntakeConstants.feedSpeed.in(RotationsPerSecond),
        _intakeFeed.getSpeed().in(RotationsPerSecond),
        0.5);

    run(_intakeFeed.feedOut());
    fastForward(Seconds.of(6));
    assertEquals(
        IntakeConstants.feedSpeed.unaryMinus().in(RotationsPerSecond),
        _intakeFeed.getSpeed().in(RotationsPerSecond),
        0.5);

    // start raising the intake, and try feeding
    run(_intakePivot.raise());
    fastForward(Seconds.of(8));

    assertEquals(0, _intakeFeed.getSpeed().in(RotationsPerSecond), 0.5);
  }
}
