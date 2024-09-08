// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RioCANIDs;

public class DriveTrain extends SubsystemBase {
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
  private final WPI_TalonSRX m_frontLeft = new WPI_TalonSRX(RioCANIDs.kDriveFrontLeft);
  private final WPI_TalonSRX m_backLeft = new WPI_TalonSRX(RioCANIDs.kDriveBackLeft);
  private final WPI_TalonSRX m_frontRight = new WPI_TalonSRX(RioCANIDs.kDriveFrontRight);
  private final WPI_TalonSRX m_backRight = new WPI_TalonSRX(RioCANIDs.kDriveBackRight);
  private final MecanumDrive m_robotDrive;
  private boolean m_fieldRelative = true;

  /**
   * This should rarely, if ever, be changed to robot relative and then for only a
   * short time.
   * 
   * @return true (the default) if driving field relative.
   */
  public boolean isFieldRelative() {
    return this.m_fieldRelative;
  }

  /**
   * @param fieldRelative true to use field relative driving (default and
   *                      preferred), or false to use robot relative driving.
   */
  public void setFieldRelative(final boolean fieldRelative) {
    this.m_fieldRelative = fieldRelative;
  }

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
    m_robotDrive.setDeadband(DriveConstants.kInputDeadband);
    m_robotDrive.setMaxOutput(DriveConstants.kMaxOutput);
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
   * The returned command will run until cancelled.
   * 
   * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is
   *               positive.
   * @param ySpeed The robot's speed along the Y axis [-1.0..1.0]. Left is
   *               positive.
   * @return a new command to drive straight on the given x,y vector without
   *         rotation.
   */
  public Command getDriveStraightCommand(final double xSpeed, final double ySpeed) {
    return run(() -> m_robotDrive.driveCartesian(
        xSpeed,
        ySpeed,
        0.0));
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
    final Rotation2d gyroAngle = m_fieldRelative
        ? m_gyro.getRotation2d()
        : DriveConstants.kGyroAngleForRobotRelative;
    m_robotDrive.driveCartesian(
        xSpeed,
        ySpeed,
        zRotation,
        gyroAngle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
