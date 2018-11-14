package com.blueshiftrobotics.ftc;

/**
 * A collection of utilities useful in the Blue Shift libraries.
 *
 * @author Gabriel Wong
 */
public class BlueShiftUtil {
    public static double getDegreeDifference(double theta_1, double theta_2) {
        double rawDifference = (theta_1 % 360) - (theta_2 % 360);

        if (rawDifference > 180) {
            return 360 - rawDifference;
        } else {
            return rawDifference;
        }
    }

    public static double getAbsDegreeDifference(double theta_1, double theta_2) {
        return Math.abs( getDegreeDifference(theta_1, theta_2) );
    }
}
