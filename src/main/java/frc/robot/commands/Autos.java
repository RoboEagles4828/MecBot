// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveTrain;

public final class Autos {
  /**
   * @param driveTrain the robot's drive train.
   * @return a command to drive straight up field to leave the starting zone.
   */
  public static Command simpleLeaveAuto(final DriveTrain driveTrain) {
    return driveTrain
        .getDriveStraightCommand(
            AutoConstants.kSimpleLeaveSpeed,
            0.0)
        .withTimeout(AutoConstants.kSimpleLeaveTimeSec);
  }

  /**
   * No instances of this static utility class.
   */
  private Autos() {
  }
}
