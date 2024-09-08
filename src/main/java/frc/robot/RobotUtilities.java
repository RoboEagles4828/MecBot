// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * A set of static utility functions. These are usually in answer to queries
 * about the field or game environment. They often involve calls to the
 * {@link DriverStation}. They are implented here as common function to ensure
 * consistent access and defaults.
 */
public class RobotUtilities {
    /**
     * This should not be called during startup since the alliance may not be known
     * yet.
     * 
     * @return the alliance for this match. The blue alliance is returned if the
     *         alliance is not known.
     */
    public static Alliance getAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue);
    }

    /** No instances of static utility class. */
    private RobotUtilities() {
    }
}
