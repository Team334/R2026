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
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

@Disabled
public class IntakeTest {
  private IntakePivot _intakePivot;
  private IntakeFeed _intakeFeed;

  private boolean _inBumpZone = false;

  @BeforeEach
  public void setup() {
    _intakePivot = new IntakePivot(() -> _inBumpZone);
    _intakeFeed = new IntakeFeed(_intakePivot.intakeLowered());

    setupTests();
  }

  @AfterEach
  public void close() {
    reset(_intakePivot, _intakeFeed);
  }

  @Test
  public void feedCorrectly() {
    run(_intakePivot.raise(), Seconds.of(1));

    // // try to feed with intake raised
    run(_intakeFeed.feedIn(), Seconds.of(1));
    assert _intakeFeed.getSpeed().equals(RotationsPerSecond.zero());

    run(_intakeFeed.feedOut(), Seconds.of(1));
    assert _intakeFeed.getSpeed().equals(RotationsPerSecond.zero());

    // try to feed with intake lowered
    run(_intakePivot.lower(), Seconds.of(2));

    run(_intakeFeed.feedIn(), Seconds.of(1));
    assertEquals(
        IntakeConstants.feedSpeed.in(RotationsPerSecond),
        _intakeFeed.getSpeed().in(RotationsPerSecond),
        0.5);

    run(_intakeFeed.feedOut(), Seconds.of(1));
    assertEquals(
        IntakeConstants.feedSpeed.unaryMinus().in(RotationsPerSecond),
        _intakeFeed.getSpeed().in(RotationsPerSecond),
        0.5);

    // start raising the intake, and try feeding
    run(_intakePivot.raise(), Seconds.of(0.5));
    assertEquals(0, _intakeFeed.getSpeed().in(RotationsPerSecond), 0.5);
  }
}
