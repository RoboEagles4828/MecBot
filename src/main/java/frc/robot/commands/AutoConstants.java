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
    
    
    static final long kShootDelay = 1000;
    // static final double[] kSourceSideLeaveSpeeds = { 0.48, 0.87 };
    static final double[] kSourceSideLeaveSpeeds = calculateSpeeds(kSourceSideLeaveXMeters, kSourceSideLeaveYMeters);
    static final double kSourceSideLeaveTimeSec = 1.5;

    static final double kAmpSideLeaveXMeters = 2.0;
    static final double kAmpSideLeaveYMeters = 1.5;
    /** (x,y) - magnitudes only, apply proper sign to y based on alliance. */
    // static final double[] kAmpSideLeaveSpeeds = { 0.8, 0.6 };
    static final double[] kAmpSideLeaveSpeeds = calculateSpeeds(kAmpSideLeaveXMeters, kAmpSideLeaveYMeters);
    static final double kAmpSideLeaveTimeSec = 1.0;

    static final double[] kmiddleLeaveSpeeds = calculateSpeeds(kAmpSideLeaveXMeters, 0);
    
    static final double kmiddleLeaveXMeters = 1.0;
    static final double kmiddleLeaveTimeSec = 1.0;
    /**
     * All values are positive. The caller will need to make sure the signs of the
     * applied speeds (especially Y sign as it varies by alliance).
     * 
     * <p>
     * TODO - Note that there is a good chance this method of calculating speeds for
     * a X by Y movement will not work well enough. It depends on this being the
     * correct equation and on a well balance drive train. Either or both could not
     * be the case. If it is off a little, tweaking the distances aboue may be okay.
     * If off by a lot, get rid of this and just switch to speed constants that can
     * be tweaked.
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
