package com.blueshiftrobotics.ftc;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    private double encoderDriveKp = 0.5;
    private int encoderDriveErrorAllowance = 10;

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
        this.motorDriveLeft = hardwareMap.get(DcMotor.class, motorDriveLeftName);
        this.motorDriveRight = hardwareMap.get(DcMotor.class, motorDriveRightName);
        this.imu = new IMU(hardwareMap, IMUName);

        motorDriveLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorDriveRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorDriveLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDriveRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Turn the robot using the REV Expansion Hub IMU using a certain `dTheta` as a parameter. The
     * sign of the dTheta will denote direction, and there is a low-level PID Controller used to
     * control the speed of the motors as the destination is approached at different rates to
     * reduce final error rates from overshoot and undershoot.
     *
     * @param dTheta The angular displacement target
     */
    public void turn(float dTheta) {
        float thetaInit;
        float thetaFinal;
        float thetaPercentError; // A Percent error is used to not have the `turn` power parameter skyrocket when dTheta is big.
        float thetaErrorInit;

        thetaInit = imu.getHeading();
        thetaFinal = thetaInit + dTheta;
        thetaErrorInit = (float)BlueShiftUtil.getDegreeDifference(thetaFinal, thetaInit);
        thetaPercentError = thetaErrorInit / thetaErrorInit;

        while (Math.abs(thetaPercentError) > turningErrorAllowance) {
            float thetaError = (float)BlueShiftUtil.getDegreeDifference(thetaFinal, thetaInit);
            thetaPercentError = thetaError / thetaErrorInit; //Modulus to protect against over 360 (wrap-around to 0) degrees.

            double turn = turnKp * thetaPercentError;

            motorDriveLeft.setPower(-turn);
            motorDriveRight.setPower(turn);
        }

        motorDriveLeft.setPower(0);
        motorDriveRight.setPower(0);
    }

    /**
     * Drive the robot to a certain encoder value on both drive train motors. The powers of each
     * motor is dependent on the percent distance not traveled yet, so, in essence, it uses a PID
     * controller.
     *
     * //TODO: Add IMU Heading Correction
     *
     * @param targetEncoderValue The encoder value to drive to
     */
    private void encoderDrive(int targetEncoderValue) {
        motorDriveLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int motorDriveLeftEncoderError, motorDriveRightEncoderError;
        double motorDriveLeftPercentEncoderError, motorDriveRightPercentEncoderError;

        while (Math.abs(motorDriveLeft.getCurrentPosition() - targetEncoderValue) > encoderDriveErrorAllowance || Math.abs(motorDriveRight.getCurrentPosition() - targetEncoderValue) > encoderDriveErrorAllowance) {
            motorDriveLeftEncoderError = targetEncoderValue - motorDriveLeft.getCurrentPosition();
            motorDriveRightEncoderError = targetEncoderValue - motorDriveRight.getCurrentPosition();
            motorDriveLeftPercentEncoderError = (double)motorDriveLeftEncoderError / (double)targetEncoderValue;
            motorDriveRightPercentEncoderError = (double)motorDriveRightEncoderError / (double)targetEncoderValue;

            double motorDriveLeftPower = encoderDriveKp * motorDriveLeftPercentEncoderError;
            double motorDriveRightPower = encoderDriveKp * motorDriveRightPercentEncoderError;

            motorDriveLeft.setPower(motorDriveLeftPower);
            motorDriveRight.setPower(motorDriveRightPower);
        }

        motorDriveLeft.setPower(0);
        motorDriveRight.setPower(0);
    }
}
