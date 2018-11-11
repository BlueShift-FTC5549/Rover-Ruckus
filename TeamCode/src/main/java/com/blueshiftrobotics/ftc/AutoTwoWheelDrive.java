package com.blueshiftrobotics.ftc;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Name Breakdown
 *   Auto - Autonomous related.
 *   TwoWheel - Uses a 2-wheel-drive robot.
 *   Drive - Includes code for moving the robot.
 *
 * Necessary Sensors
 *   IMU (REV Expansion Hub)
 *
 * The autonomous library for the driving of a two wheeled robot during the thirty second autonomous
 * phase. Using the IMU, the robot can turn precisely to a degree translation using the `turn`
 * method.
 *
 * @author Gabriel Wong
 */
public class AutoTwoWheelDrive {
    private DcMotor motorDriveLeft;
    private DcMotor motorDriveRight;
    private IMU imu;

    private double turnKp = 0.5;
    private float turningErrorAllowance = 2; //In Degrees

    /**
     * Create a new instance of a two wheel drive library using the current hardware map and names
     * of components of interest.
     *
     * @param hardwareMap Current hardware map
     * @param motorDriveLeftName Name of the left drive motor
     * @param motorDriveRightName Name of the right drive motor
     * @param IMUName Name of the IMU sensor on the REV Expansion Hub
     */
    public AutoTwoWheelDrive(HardwareMap hardwareMap, String motorDriveLeftName, String motorDriveRightName, String IMUName) {
        this.motorDriveLeft = motorDriveLeft;
        this.motorDriveRight = motorDriveRight;

        motorDriveLeft = hardwareMap.get(DcMotor.class, motorDriveLeftName);
        motorDriveRight = hardwareMap.get(DcMotor.class, motorDriveRightName);
        imu = new IMU(hardwareMap, IMUName);
    }

    /**
     * Turn the robot using the REV Expansion Hub IMU using a certain `dTheta` as a parameter. The
     * sign of the dTheta will denote direction, and there is a low-level PID Controller used to
     * control the speed of the motors as the destination is approached at different rates to
     * reduce final error rates from overshoot and undershoot.
     *
     * @param dTheta The angular displacement target
     */
    private void turn(float dTheta) {
        float thetaInit;
        float thetaFinal;
        float thetaPercentError; // A Percent error is used to not have the `turn` power parameter skyrocket when dTheta is big.
        float thetaErrorInit;

        thetaInit = imu.getHeading();
        thetaFinal = thetaInit + dTheta;
        thetaErrorInit = thetaFinal - thetaInit;
        thetaPercentError = thetaErrorInit / thetaErrorInit;

        //TODO: Use MOD to fix errors when the IMU jumps from 0 to 360 etc
        while (Math.abs(thetaPercentError) > turningErrorAllowance) {
            thetaPercentError = (thetaFinal - thetaInit) / thetaErrorInit;

            double turn = turnKp * thetaPercentError;

            motorDriveLeft.setPower(-turn);
            motorDriveRight.setPower(turn);
        }
    }
}
