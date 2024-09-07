// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RioCANIDs;

public class DriveTrain extends SubsystemBase {
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
  private final TalonFX m_frontLeft = new TalonFX(RioCANIDs.kDriveFrontLeft);
  private final TalonFX m_backLeft = new TalonFX(RioCANIDs.kDriveBackLeft);
  private final TalonFX m_frontRight = new TalonFX(RioCANIDs.kDriveFrontRight);
  private final TalonFX m_backRight = new TalonFX(RioCANIDs.kDriveBackRight);
  private final MecanumDrive m_robotDrive;

  /**
   * Creates a new DriveTrain with the default input deadband and maximum output
   * as defined at
   * {@link edu.wpi.first.wpilibj.drive.RobotDriveBase#kDefaultDeadband} and
   * {@link edu.wpi.first.wpilibj.drive.RobotDriveBase#kDefaultMaxOutput}.
   */
  public DriveTrain() {
    // Left side not inverted.
    m_frontLeft.setInverted(false);
    m_backLeft.setInverted(false);
    // Right side inverted.
    m_frontRight.setInverted(true);
    m_backRight.setInverted(true);
    m_robotDrive = new MecanumDrive(m_frontLeft, m_backLeft, m_frontRight, m_backRight);
  }

  /**
   * Creates a new DriveTrain with a custom input deadband and maximum output.
   */
  public DriveTrain(final double inputDeadband, final double maxOutput) {
    this();
    m_robotDrive.setDeadband(inputDeadband);
    m_robotDrive.setMaxOutput(maxOutput);
  }

  /**
   * Returns a command for cartesian drive. The command requires this subsystem.
   * All driving is field relative. The configured (or default) deadbands and
   * maximums will be enforced.
   * 
   * @param xSpeedSupplier    a supplier of a stream of X axis inputs [-1.0..1.0].
   *                          Forward is positive.
   * @param ySpeedSupplier    a supplier of a stream of Y axis inputs [-1.0..1.0].
   *                          Left is positive.
   * @param zRotationSupplier a supplier of a stream of rotation rates around the
   *                          Z axis [-1.0..1.0]. Counterclockwise is positive.
   * @return a newly created drive command.
   */
  public Command getDriveCommand(
      final DoubleSupplier xSpeedSupplier,
      final DoubleSupplier ySpeedSupplier,
      final DoubleSupplier zRotationSupplier) {
    return run(() -> driveCartesian(
        xSpeedSupplier.getAsDouble(),
        ySpeedSupplier.getAsDouble(),
        zRotationSupplier.getAsDouble()));
  }

  /**
   * @return a simple command to just leave the starting zone during auto.
   */
  public Command leaveStartingZoneAutoCommand() {
    return run(() -> m_robotDrive
        .driveCartesian(
            DriveConstants.kSimpleLeaveAutoSpeed,
            0.0,
            0.0))
        .withTimeout(DriveConstants.kSimpleLeaveAutoTimeSec);
  }

  /**
   * The configured (or default) deadbands and maximums will be enforced.
   *
   * @param xSpeed    The robot's speed along the X axis [-1.0..1.0]. Forward is
   *                  positive.
   * @param ySpeed    The robot's speed along the Y axis [-1.0..1.0]. Left is
   *                  positive.
   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0].
   *                  Counterclockwise is positive.
   */
  void driveCartesian(final double xSpeed, final double ySpeed, final double zRotation) {
    m_robotDrive.driveCartesian(
        xSpeed,
        ySpeed,
        zRotation,
        m_gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
