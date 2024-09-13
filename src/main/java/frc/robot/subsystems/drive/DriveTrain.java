// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    configureMotor(m_frontLeft);
    configureMotor(m_backLeft);
    configureMotor(m_frontRight);
    configureMotor(m_backRight);
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
   * This command should be run once after starting the robot to properly set the
   * gyro angle offset. As the very first thing in autonomous would be a
   * reasonable choice.
   * 
   * <p>
   * Note that the NavX gyro is clockwise positive. That is why the value is
   * negated. If we change to a Pigeon, the negation will need to be removed.
   * 
   * @param startupAngle start up angle in standard FRC counter clockwise positive
   *                     orientation.
   * @return a newly created set startup angle command.
   */
  public Command getSetStartupAngleCommand(final double startupAngle) {
    return runOnce(() -> m_gyro.setAngleAdjustment(-startupAngle));
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
    return run(() -> driveCartesian(
        xSpeed,
        ySpeed,
        0.0));
  }

  /**
   * This command MUST only be used when pointing straight up the field (intake
   * away from driver). Use the field sidewall or other structure to get straight
   * before running this.
   * 
   * @return a newly created reset gyro command.
   */
  public Command getResetGyroCommand() {
    return runOnce(() -> {
      // Resets gyro yaw but not the angle adjustment.
      m_gyro.reset();
      // Reset the angle adjustment too.
      m_gyro.setAngleAdjustment(0.0);
    });
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
    /*
     * Note that the NavX getRotation2d API is the one place where it returns the
     * angle in CCW+. No negation needed here.
     */
    final Rotation2d gyroAngle = m_fieldRelative
        ? m_gyro.getRotation2d()
        : DriveConstants.kGyroAngleForRobotRelative;
    m_robotDrive.driveCartesian(
        xSpeed,
        ySpeed,
        zRotation,
        gyroAngle);
  }

  /**
   * Used to configure each motor to the same initial conditions.
   * 
   * @param motor the motor to configure.
   */
  private void configureMotor(final WPI_TalonSRX motor) {
    motor.setInverted(false);
    motor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Heading", -m_gyro.getAngle());
    SmartDashboard.putBoolean("Field Relative", m_fieldRelative);
  }
}
