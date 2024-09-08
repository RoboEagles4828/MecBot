// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;

public final class DriveConstants {
    static final Rotation2d kGyroAngleForRobotRelative = new Rotation2d();
    static final double kInputDeadband = RobotDriveBase.kDefaultDeadband;
    static final double kMaxOutput = RobotDriveBase.kDefaultMaxOutput;
}
