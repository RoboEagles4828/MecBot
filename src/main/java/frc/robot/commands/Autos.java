// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotUtilities;
import frc.robot.subsystems.drive.DriveTrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

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
                    AutoConstants.kSimpleLeaveSpeed, // TODO: Ask moore if this should be reversed (for straight taxi) 
                    0.0)
                .withTimeout(AutoConstants.kSimpleLeaveTimeSec));
  }

  /**
   * Place the robot on the field against the subwoofer on the source side with
   * the intake pointing away from the subwoofer. This auto will shoot and then
   * leave the starting zone.
   * 
   * @param driveTrain the robot's drive train. // TODO add the shooter and intake
   * @return an auto source side command to score and leave the starting zone.
   */
  public static Command sourceSideScoreAndLeave(final DriveTrain driveTrain, final Shooter shooter, final Intake intake) {
    final double ySign = RobotUtilities.getAlliance() == Alliance.Red ? -1.0 : 1.0;
    return driveTrain
        .getSetStartupAngleCommand(Constants.StartingAngle.SUBWOOFER_SOURCE_SIDE.getOffsetDegrees())
        .andThen(shooter.getShootCommand()).alongWith(intake.getShootCommand())
        .andThen(new WaitCommand(AutoConstants.kShootDelay))
        .andThen(shooter.getStopCommand()).alongWith(intake.getStopCommand())
        .andThen(
            driveTrain
                .getDriveStraightCommand(
                    -AutoConstants.kSourceSideLeaveSpeeds[0], // This is inverted to make the bot go backwards
                    AutoConstants.kSourceSideLeaveSpeeds[1] * ySign)
                .withTimeout(AutoConstants.kSourceSideLeaveTimeSec));
  }

  /**
   * Place the robot on the field against the subwoofer on the amp side with
   * the intake pointing away from the subwoofer. This auto will shoot and then
   * leave the starting zone.
   * 
   * @param driveTrain the robot's drive train. // TODO add the shooter and intake
   * @return an auto amp side command to score and leave the starting zone.
   */
  public static Command ampSideScoreAndLeave(final DriveTrain driveTrain, final Shooter shooter, final Intake intake) {
    final double ySign = RobotUtilities.getAlliance() == Alliance.Blue ? -1.0 : 1.0;
    return driveTrain
        .getSetStartupAngleCommand(Constants.StartingAngle.SUBWOOFER_AMP_SIDE.getOffsetDegrees())
        .andThen(shooter.getShootCommand()).alongWith(intake.getShootCommand())
        .andThen(new WaitCommand(AutoConstants.kShootDelay))
        .andThen(shooter.getStopCommand()).alongWith(intake.getStopCommand())
        .andThen(
            driveTrain
                .getDriveStraightCommand(
                    -AutoConstants.kAmpSideLeaveSpeeds[0], // This is inverted to make the bot go backwards
                    AutoConstants.kAmpSideLeaveSpeeds[1] * ySign)
                .withTimeout(AutoConstants.kAmpSideLeaveTimeSec));
  }

  /**
   * Place the robot on the field against the subwoofer on the amp side with
   * the intake pointing away from the subwoofer. This auto will shoot and then
   * leave the starting zone.
   * 
   * @param driveTrain the robot's drive train. // TODO add the shooter and intake
   * @return an auto amp side command to score and leave the starting zone.
   */
  public static Command middleScoreAndLeave(final DriveTrain driveTrain, final Shooter shooter, final Intake intake) {
    // final double ySign = RobotUtilities.getAlliance() == Alliance.Blue ? -1.0 : 1.0;
    return driveTrain
        .getSetStartupAngleCommand(Constants.StartingAngle.SUBWOOFER_AMP_SIDE.getOffsetDegrees())
        .andThen(shooter.getShootCommand()).alongWith(intake.getShootCommand())
        .andThen(new WaitCommand(AutoConstants.kShootDelay))
        .andThen(shooter.getStopCommand()).alongWith(intake.getStopCommand())
        .andThen(
            driveTrain
                .getDriveStraightCommand(
                    -AutoConstants.kmiddleLeaveSpeeds[0], // This is inverted to make the bot go backwards
                    0)
                .withTimeout(AutoConstants.kmiddleLeaveTimeSec));
  }

  /**
   * No instances of this static utility class.
   */
  private Autos() {
  }
}
