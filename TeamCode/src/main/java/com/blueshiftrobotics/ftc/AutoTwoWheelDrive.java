package com.blueshiftrobotics.ftc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
 * @version 0.8
 * @author Gabriel Wong
 */
public class AutoTwoWheelDrive {
    private DcMotor motorDriveLeft;
    private DcMotor motorDriveRight;
    private IMU imu;

    private Telemetry telemetry;
    private LinearOpMode opMode;

    private ElapsedTime elapsedTime = new ElapsedTime();

    //Turning Constants
    private static final float turnKp = (float)0.85;
    private static final float turningErrorAllowance = 1; //In Degrees
    private static final float turningPowerThreshold = (float)0.05; //TODO: Tune this constant.

    //Encoder Constants
    private static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final float      ENCODER_DRIVE_Kp        = (float)0.5;
    private static final int        ENCODER_DRIVE_ERROR_ALLOWANCE = 10;
    private static final float      ENCODER_DRIVE_POWER_THRESHOLD = (float)0.1; //TODO: Tune this constant.

    /**
     * Create a new instance of a two wheel drive library using the current hardware map and names
     * of components of interest.
     *
     * @param opMode The Linear Op Mode that constructs this library.
     * @param motorDriveLeftName Name of the left drive motor
     * @param motorDriveRightName Name of the right drive motor
     * @param IMUName Name of the IMU sensor on the REV Expansion Hub
     */
    public AutoTwoWheelDrive(LinearOpMode opMode, String motorDriveLeftName, String motorDriveRightName, String IMUName) {
        this.motorDriveLeft = opMode.hardwareMap.get(DcMotor.class, motorDriveLeftName);
        this.motorDriveRight = opMode.hardwareMap.get(DcMotor.class, motorDriveRightName);
        this.imu = new IMU(opMode.telemetry, opMode.hardwareMap, IMUName);

        motorDriveLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorDriveRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorDriveLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDriveRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.telemetry = opMode.telemetry;
        this.opMode = opMode;
    }

    /**
     * Turn the robot using the REV Expansion Hub IMU using a certain `dTheta` as a parameter. The
     * sign of the dTheta will denote direction, and there is a low-level PID Controller used to
     * control the speed of the motors as the destination is approached at different rates to
     * reduce final error rates from overshoot and undershoot.
     *
     * //TODO: Fix zero at beginning at end
     * //TODO: Add runtime timeout
     *
     * @param dTheta The angular displacement target
     */
    public void turn(float dTheta) {
        float thetaInit;
        float thetaTarget;
        float thetaCurrent;
        float thetaPercentError = 1; // A Percent error is used to not have the `turn` power parameter skyrocket when dTheta is big.

        thetaInit = imu.getHeading();

        thetaTarget = (thetaInit + dTheta) % 360;

        while (Math.abs(thetaPercentError * dTheta) > turningErrorAllowance && opMode.opModeIsActive()) {
            thetaCurrent = imu.getHeading();

            float thetaError = (float)BlueShiftUtil.getDegreeDifference(thetaTarget, thetaCurrent);

            thetaPercentError = thetaError / dTheta;

            float turningPower = turnKp * thetaPercentError * (1 - thetaPercentError) + turningPowerThreshold;

            motorDriveLeft.setPower(turningPower);
            motorDriveRight.setPower(-turningPower);

            telemetry.addData("Percent Error", thetaPercentError);
            telemetry.addData("Current Heading", thetaCurrent);
            telemetry.addData("Target Heading", thetaTarget);
            telemetry.update();
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
     * @param targetDistance The distance in inches to travel
     * @param errorTimeout The maximum seconds to run the loop for
     */
    public void encoderDrive(double targetDistance, double errorTimeout) {
        motorDriveLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorDriveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDriveRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Encoder Driving");
        telemetry.addData("Path",  "Starting at %7d :%7d", motorDriveLeft.getCurrentPosition(), motorDriveRight.getCurrentPosition());
        telemetry.update();

        int encoderTarget = (int)(targetDistance * COUNTS_PER_INCH);
        int motorDriveLeftEncoderError, motorDriveRightEncoderError;
        double motorDriveLeftPercentEncoderError, motorDriveRightPercentEncoderError;

        elapsedTime.reset();

        while (elapsedTime.seconds() <= errorTimeout && Math.abs(motorDriveLeft.getCurrentPosition() - encoderTarget) > ENCODER_DRIVE_ERROR_ALLOWANCE || Math.abs(motorDriveRight.getCurrentPosition() - encoderTarget) > ENCODER_DRIVE_ERROR_ALLOWANCE && opMode.opModeIsActive()) {
            motorDriveLeftEncoderError = encoderTarget - motorDriveLeft.getCurrentPosition();
            motorDriveRightEncoderError = encoderTarget - motorDriveRight.getCurrentPosition();
            motorDriveLeftPercentEncoderError = (double)motorDriveLeftEncoderError / (double)encoderTarget;
            motorDriveRightPercentEncoderError = (double)motorDriveRightEncoderError / (double)encoderTarget;

            double motorDriveLeftPower = ENCODER_DRIVE_Kp * motorDriveLeftPercentEncoderError * (1 - motorDriveLeftPercentEncoderError) + ENCODER_DRIVE_POWER_THRESHOLD;
            double motorDriveRightPower = ENCODER_DRIVE_Kp * motorDriveRightPercentEncoderError * (1 - motorDriveRightPercentEncoderError) + ENCODER_DRIVE_POWER_THRESHOLD;

            telemetry.addData("Left Encoder", motorDriveLeft.getCurrentPosition());
            telemetry.addData("Right Encoder", motorDriveLeft.getCurrentPosition());

            motorDriveLeft.setPower(motorDriveLeftPower);
            motorDriveRight.setPower(motorDriveRightPower);
        }

        motorDriveLeft.setPower(0);
        motorDriveRight.setPower(0);

        motorDriveLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDriveRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Encoder Driving COMPLETE");
        telemetry.update();
    }
}
