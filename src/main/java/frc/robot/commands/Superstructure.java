// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.InputStream;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.IntakeFeed;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.Shooting;

/**
 * Contains factory methods for commands that involve the whole superstructure of the robot (require
 * multiple subsystems).
 */
public class Superstructure {
  private Climb _climb;
  private Hopper _hopper;
  private IntakeFeed _intakeFeed;
  private IntakePivot _intakePivot;
  private LED _led;
  private Shooter _shooter;
  private Swerve _swerve;

  private Shooting _shooting;

  public Superstructure(
      Climb climb,
      Hopper hopper,
      IntakeFeed intakeFeed,
      IntakePivot intakePivot,
      LED led,
      Shooter shooter,
      Swerve swerve,
      Shooting shooting) {
    _climb = climb;
    _hopper = hopper;
    _intakeFeed = intakeFeed;
    _intakePivot = intakePivot;
    _led = led;
    _shooter = shooter;
    _swerve = swerve;
    _shooting = shooting;
  }

  public Command shoot(InputStream velX, InputStream velY) {
    return parallel(
        _shooter.shoot(() -> _shooting.getFlywheelSpeed()),
        _hopper.feed(() -> _shooting.getRollerSpeed(), () -> _shooting.getFloorSpeed()),
        _intakePivot.pivotRaise(),
        _swerve.driveFacing(velX, velY, () -> _shooting.getShotHeading()));
  }
}
