/* Copyright (c) 2018 Blue Shift Robotics
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package com.blueshiftrobotics.ftc;

/**
 * A collection of utilities useful in the Blue Shift libraries.
 *
 * @author Gabriel Wong
 */
public class BlueShiftUtil {

    /**
     * @param theta_1 The first angle
     * @param theta_2 The second angle
     * @return the difference between to angles in the range [-180, 180].
     */
    public static double getDegreeDifference(double theta_1, double theta_2) {
        double rawDifference = theta_2 - theta_1;

        if (rawDifference > 180) {
            return rawDifference - 360;
        } else if (rawDifference < -180) {
            return rawDifference + 360;
        } else {
            return rawDifference;
        }
    }

    /**
     * Calculate the Mean Absolute Error of a given array from a given target value. Useful for
     * getting the average deviation from some target value for every value in an array. Modified
     * by also taking the absolute value of the array value and target before computing difference.
     *
     * @param array Array of values
     * @param target Target Value
     * @return Mean Absolute Eror
     */
    public static double modifiedMeanAbsoluteError(int[] array, int target) {
        double sum = 0;
        double signedSum = 0;

        //Motor indeces 1 and 2 move backward compared to 0 and 4, so they are negative
        sum += Math.abs(array[0] - target);
        sum += Math.abs(-array[1] - target);
        sum += Math.abs(-array[2] - target);
        sum += Math.abs(array[3] - target);
        signedSum += array[0] - target;
        signedSum += -array[1] - target;
        signedSum += -array[2] - target;
        signedSum += array[3] - target;

        return Math.signum(signedSum) * (sum / ((double)array.length));
    }

    /**
     * Perform the sigmoid function for a given value `x`. Equal to (1 / (1+e^(-x))). Useful because
     * it is a smooth logistic curve.
     *
     * @param x Value to sigmoid.
     * @return Sigmoid value of x.
     */
    public static double sigmoid(double x) {
        return Math.pow(1.0 + Math.exp(-x), -1.0);
    }

    /**
     * Perform the derivative of the sigmoid function for a given value `x`. Equal to sigmoid *
     * (1 - sigmoid). Shifted to the right by 0.5 to have the peak at 0.5, `x` is scaled by 5 to
     * compress the curve horizontally, and the entire function is multiplied by 3.0 to have `y`
     * peak at 0.75.
     *
     * @param x
     * @return
     */
    public static double modifiedSigmoidPrime(double x) {
        return 3.0*sigmoid(5.0*(x-0.5)) * (1.0 - sigmoid(5.0*(x-0.5)));
    }
}
