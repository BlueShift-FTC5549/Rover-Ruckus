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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * A customized IMU object class that is somewhat an extension of BNO055IMU class. To simplify code
 * and make it more readable in the Autonomous libraries, the BNO055IMU is initialized here and the
 * heading retrieval method is collapsed to a simple `getHeading()` method.
 *
 * @version 1.0
 * @author Gabriel Wong
 */
public class IMU {
    private BNO055IMU revIMU;

    /**
     * Create a new BNO055IMU with preset parameters, the hardware component being added with a
     * HardwareMap parameter and the robot configuration name for the IMU's I2C bus.
     *
     * @param hardwareMap The robot controller's hardware map
     * @param IMUName The name of the BNO055IMU in the robot controller.
     */
    public IMU(HardwareMap hardwareMap, String IMUName) {
        try {
            revIMU = hardwareMap.get(BNO055IMU.class, IMUName);

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = IMUName + "AdafruitIMUCalibration.json";
            parameters.loggingEnabled = true;
            parameters.loggingTag = IMUName;

            revIMU.initialize(parameters);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * Create a new BNO055IMU with preset parameters, the hardware component being added with a
     * HardwareMap parameter and the robot configuration name for the IMU's I2C bus.
     *
     * @param telemetry The driver station telemetry for status communication.
     * @param hardwareMap The robot controller's hardware map.
     * @param IMUName The name of the BNO055IMU in the robot controller
     */
    public IMU(Telemetry telemetry, HardwareMap hardwareMap, String IMUName) {
        try {
            revIMU = hardwareMap.get(BNO055IMU.class, IMUName);

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
            parameters.loggingEnabled = true;
            parameters.loggingTag = IMUName;

            revIMU.initialize(parameters);
        } catch (Exception e) {
            e.printStackTrace();

            telemetry.addData("IMU Status", "ERROR ERROR ERROR");
        }
    }

    /**
     * @return the current orientation from the BNO055IMU.
     */
    public Orientation getOrientation() {
        return revIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    /**
     * Retrieve the current compass heading from the BNO055IMU using a preset selected axis. 180
     * must be added to the return value so that the range changes from (-180,180) to (0,360).
     *
     * @return the current magnetic heading
     */
    public float getHeading() {
        return revIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + 180;
    }

    public void initAccelerationLogging() {
        revIMU.startAccelerationIntegration(new Position(), new Velocity(), 250);
    }

    public Acceleration getAcceleration() {
        return revIMU.getAcceleration();
    }
    public Velocity getVelocity() { return revIMU.getVelocity(); }
}
