// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveTrain;

public final class Autos {
  /**
   * Place the robot on the field against the alliance wall with the intake
   * pointing away from the wall. This auto will do nothing but set the starting
   * angle.
   * 
   * @param driveTrain the robot's drive train.
   * @return an auto command to do nothing.
   */
  public static Command doNothingAuto(final DriveTrain driveTrain) {
    return driveTrain
        .getSetStartupAngleCommand(Constants.StartingAngle.SQUARE.getOffsetDegrees());
  }

  /**
   * Place the robot on the field against the alliance wall with the intake
   * pointing away from the wall. This auto will simply drive forward to leave the
   * starting zone.
   * 
   * @param driveTrain the robot's drive train.
   * @return an auto command to drive straight up field to leave the starting
   *         zone.
   */
  public static Command simpleLeaveAuto(final DriveTrain driveTrain) {
    return driveTrain
        .getSetStartupAngleCommand(Constants.StartingAngle.SQUARE.getOffsetDegrees())
        .andThen(
            driveTrain
                .getDriveStraightCommand(
                    AutoConstants.kSimpleLeaveSpeed,
                    0.0)
                .withTimeout(AutoConstants.kSimpleLeaveTimeSec));
  }

  /**
   * No instances of this static utility class.
   */
  private Autos() {
  }
}
