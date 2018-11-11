package com.blueshiftrobotics.ftc;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * A customized IMU object class that is somewhat an extension of BNO055IMU class. To simplify code
 * and make it more readable in the Autonomous libraries, the BNO055IMU is initialized here and the
 * heading retrieval method is collapsed to a simple `getHeading()` method.
 *
 * @Author Gabriel Wong
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
        revIMU = hardwareMap.get(BNO055IMU.class, IMUName);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        revIMU.initialize(parameters);
    }

    /**
     * Retrieve the current compass heading from the BNO055IMU using a preset selected axis.
     *
     * @return the current magnetic heading
     */
    public float getHeading() {
        //TODO: If the axes of the orientation of the REV hub changes, we must change the Axes Order to get the correct angle.
        return revIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
}
