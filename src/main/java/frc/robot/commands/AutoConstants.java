// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

public final class AutoConstants {
    static final double kSimpleLeaveSpeed = 0.6;
    static final double kSimpleLeaveTimeSec = 1.0;

    static final double kSourceSideLeaveXMeters = 2.0;
    static final double kSourceSideLeaveYMeters = 3.6;
    /** (x,y) - magnitudes only, apply proper sign to y based on alliance. */
    static final double[] kSourceSideLeaveSpeeds = calculateSpeeds(kSourceSideLeaveXMeters, kSourceSideLeaveYMeters);
    static final double kSourceSideLeaveTimeSec = 2.0;

    static final double kAmpSideLeaveXMeters = 2.0;
    static final double kAmpSideLeaveYMeters = 1.5;
    /** (x,y) - magnitudes only, apply proper sign to y based on alliance. */
    static final double[] kAmpSideLeaveSpeeds = calculateSpeeds(kAmpSideLeaveXMeters, kAmpSideLeaveYMeters);
    static final double kAmpSideLeaveTimeSec = 1.5;

    /**
     * All values are positive. The caller will need to make sure the signs of the
     * applied speeds (especially Y sign as it varies by alliance).
     * 
     * @param xMeters the number of meters to move in the X direction.
     * @param yMeters the number of meters to move in the Y direction.
     * @return a array of two doubles (x,y) of the speeds to use.
     */
    private static double[] calculateSpeeds(final double xMeters, final double yMeters) {
        final double hypotenuseMagnitude = Math.sqrt((xMeters * xMeters) + (yMeters * yMeters));
        return new double[] { xMeters / hypotenuseMagnitude, yMeters / hypotenuseMagnitude };
    }
}
