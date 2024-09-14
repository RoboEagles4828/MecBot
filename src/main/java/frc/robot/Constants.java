// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  /**
   * Constants for the driver and operator interface.
   */
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriverControllerDeadband = 0.1;
  }

  /**
   * Ids on the RIO CAN bus.
   */
  public static class RioCANIDs {
    public static final int kDriveFrontLeft = 1;
    public static final int kDriveBackLeft = 2;
    public static final int kDriveFrontRight = 3;
    public static final int kDriveBackRight = 4;

    public static final int kShooterRight = 5;
    public static final int kShooterLeft = 6;
  }

  /**
   * Ids on the PWM boards
   */
  public static class PWMIDs {
    public static final int kIntake = 9;
  }


  /**
   * Digital IO ids
   */
  public static class DigitalIDs {
    public static final int kBeamBreak = 9;
  }

  /**
   * There are multiple coordinate systems on an FRC field. For field relative
   * driving, the coordinate system (as viewed from the driver station) point
   * (0,0) is at the intersection of the field side planes of the alliance wall
   * and the side wall to the driver's right. The X direction is positive moving
   * away from the driver and the Y direction is positive from right to left.
   * Therefore, all robot positions on the field are always positive for both X
   * and Y. Rotation angles are measured counter-clockwise positive (CCW+).
   * 
   * <p>
   * When gyroscopes, like the one on the robot, boot up and initialize, they
   * choose their current orientation in all three axes as 0 degrees. This works
   * fine without compensation if the robot is placed on the field facing forward
   * (pointing X positive) and then turned on. However, there will be times where
   * we want to start at some other angle and gyro compensation must be used. This
   * enumeration defines the common cases for this year's game.
   * 
   * <p>
   * For this robot, the front is the intake and the shooter is at the back.
   */
  public enum StartingAngle {
    /** Against endwall or subwoofer front pointing away from wall. */
    SQUARE(0.0, 0.0),
    /** Against subwoofer on the source side pointing away from subwoofer. */
    SUBWOOFER_SOURCE_SIDE(61.775, -61.775),
    /** Against subwoofer on the amp side pointing away from subwoofer. */
    SUBWOOFER_AMP_SIDE(-61.775, 61.775);

    private final double m_redOffsetDegrees;
    private final double m_blueOffsetDegrees;

    private StartingAngle(final double redOffsetDegrees, final double blueOffsetDegrees) {
      m_redOffsetDegrees = redOffsetDegrees;
      m_blueOffsetDegrees = blueOffsetDegrees;
    }

    /**
     * This should not be called during startup since the alliance may not be known
     * yet.
     * 
     * @return the offset for this starting angle for the current alliance. The blue
     *         side angle is returned if the alliance is not known.
     */
    public double getOffsetDegrees() {
      return RobotUtilities.getAlliance() == Alliance.Blue
          ? m_blueOffsetDegrees
          : m_redOffsetDegrees;
    }
  }
}
