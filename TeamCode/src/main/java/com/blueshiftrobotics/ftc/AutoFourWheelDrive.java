package com.blueshiftrobotics.ftc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
 * The autonomous library for the hasAborted of a two wheeled robot during the thirty second autonomous
 * phase. Using the IMU, the robot can turn precisely to a degree translation using the `turn`
 * method.
 *
 * @version 0.8
 * @author Gabriel Wong
 */
public class AutoFourWheelDrive {
    //Robot Motors
    private DcMotor motorDriveLeftBack;
    private DcMotor motorDriveLeftFront;
    private DcMotor motorDriveRightBack;
    private DcMotor motorDriveRightFront;

    //Important Objects to Manipulate
    private IMU imu;
    private Telemetry telemetry;
    private LinearOpMode opMode;
    private ElapsedTime elapsedTime = new ElapsedTime();

    //Instance Variables
    private static boolean hasAborted = false;

    //Turning Constants
    private static final float TURN_Kp = (float)0.3;
    private static final float TURN_ERROR_ALLOWANCE = (float)1; //In Degrees
    private static final float TURN_POWER_THRESHOLD = (float)0.3; //TODO: Tune this constant.
    private static final float GAUSSIAN_REDUCTION = (float)(-4 * Math.log(TURN_POWER_THRESHOLD));

    //Encoder Constants
    private static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 6.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final float      ENCODER_DRIVE_Kp        = (float)0.8; //TODO: Tune this constant.
    private static final int        ENCODER_DRIVE_ERROR_ALLOWANCE = 15; //TODO: Tune this constant.
    private static final float      ENCODER_DRIVE_POWER_THRESHOLD = (float)0.2; //TODO: Tune this constant.

    /**
     * Create a new instance of a two wheel drive library using the current hardware map and names
     * of components of interest.
     *
     * @param opMode The Linear Op Mode that constructs this library.
     * @param motorDriveLeftName Name of the left drive motor
     * @param motorDriveRightName Name of the right drive motor
     * @param IMUName Name of the IMU sensor on the REV Expansion Hub
     */
    public AutoFourWheelDrive(LinearOpMode opMode, String motorDriveLeftName, String motorDriveRightName, String IMUName) {
        this.motorDriveLeftBack = opMode.hardwareMap.get(DcMotor.class, motorDriveLeftName + "Back");
        this.motorDriveLeftFront = opMode.hardwareMap.get(DcMotor.class, motorDriveLeftName + "Front");
        this.motorDriveRightBack = opMode.hardwareMap.get(DcMotor.class, motorDriveRightName + "Back");
        this.motorDriveRightFront = opMode.hardwareMap.get(DcMotor.class, motorDriveRightName + "Front");
        this.imu = new IMU(opMode.telemetry, opMode.hardwareMap, IMUName);

        motorDriveLeftBack.setDirection(DcMotor.Direction.FORWARD);
        motorDriveLeftFront.setDirection(DcMotor.Direction.FORWARD);
        motorDriveRightBack.setDirection(DcMotor.Direction.REVERSE);
        motorDriveRightFront.setDirection(DcMotor.Direction.REVERSE);

        motorDriveLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDriveLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDriveRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDriveRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.telemetry = opMode.telemetry;
        this.opMode = opMode;
    }

    /**
     * Turn the robot using the REV Expansion Hub IMU using a certain `dTheta` as a parameter. The
     * sign of the dTheta will denote direction, and there is a low-level PID Controller used to
     * control the speed of the motors as the destination is approached at different rates to
     * reduce final error rates from overshoot and undershoot.
     *
     * @param dTheta The angular displacement target
     */
    public void turn(float dTheta, double secondsTimeout) {
        hasAborted = true;

        float thetaInit = imu.getHeading();
        float thetaTarget = (thetaInit + dTheta) % 360;
        float thetaCurrent = imu.getHeading();
        float thetaError = (float)BlueShiftUtil.getDegreeDifference(thetaCurrent, thetaTarget);
        float thetaPercentError = thetaError / dTheta; // A Percent error is used to not have the `turn` power parameter skyrocket when dTheta is big.

        elapsedTime.reset();

        while (Math.abs(thetaPercentError * dTheta) > TURN_ERROR_ALLOWANCE
                && opMode.opModeIsActive()
                && elapsedTime.seconds() <= secondsTimeout
                && hasAborted) {

            thetaCurrent = imu.getHeading();
            thetaError = (float)BlueShiftUtil.getDegreeDifference(thetaCurrent, thetaTarget);
            thetaPercentError = thetaError / dTheta;

            float turningPower = (float) (TURN_Kp * Math.exp(-GAUSSIAN_REDUCTION * Math.pow(Range.clip(Math.abs(thetaPercentError), 0, 1) - (1.0/2.0), 2.0)));

            float leftPower = Math.signum(thetaPercentError) * Math.signum(dTheta) * turningPower;
            float rightPower = -leftPower;

            motorDriveLeftBack.setPower(leftPower);
            motorDriveLeftFront.setPower(leftPower);
            motorDriveRightBack.setPower(rightPower);
            motorDriveRightBack.setPower(rightPower);

            telemetry.addData("Theta Error", thetaError);
            telemetry.addData("Percent Error", thetaPercentError);
            telemetry.addData("Current Heading", thetaCurrent);
            telemetry.addData("Target Heading", thetaTarget);
            telemetry.addData("Motor Power", turningPower);
            telemetry.update();
        }

        abortMotion();
    }

    /**
     * Drive the robot to a certain encoder value on both drive train motors. The powers of each
     * motor is dependent on the percent distance not traveled yet, so, in essence, it uses a PID
     * controller.
     *
     * //TODO: Add IMU Heading Correction
     *
     * @param targetDistance The distance in inches to travel
     * @param secondsTimeout The maximum seconds to run the loop for
     */
    public void encoderDrive(double targetDistance, double secondsTimeout) {
        hasAborted = true;

        motorDriveLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorDriveLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDriveLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDriveRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDriveRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Encoder Driving");
        telemetry.addData("Path",  "Starting at %7d : %7d", motorDriveLeftBack.getCurrentPosition(), motorDriveRightBack.getCurrentPosition());
        telemetry.update();

        int encoderTarget = (int)(targetDistance * COUNTS_PER_INCH);
        int motorDriveLeftEncoderError, motorDriveRightEncoderError;
        double motorDriveLeftPercentEncoderError, motorDriveRightPercentEncoderError;

        elapsedTime.reset();

        while (elapsedTime.seconds() <= secondsTimeout
                && ( Math.abs(motorDriveLeftBack.getCurrentPosition() - encoderTarget) > ENCODER_DRIVE_ERROR_ALLOWANCE
                     || Math.abs(motorDriveRightBack.getCurrentPosition() - encoderTarget) > ENCODER_DRIVE_ERROR_ALLOWANCE )
                && opMode.opModeIsActive()
                && !hasAborted) {

            motorDriveLeftEncoderError = encoderTarget - motorDriveLeftBack.getCurrentPosition();
            motorDriveRightEncoderError = encoderTarget - motorDriveRightBack.getCurrentPosition();
            motorDriveLeftPercentEncoderError = (double)motorDriveLeftEncoderError / (double)encoderTarget;
            motorDriveRightPercentEncoderError = (double)motorDriveRightEncoderError / (double)encoderTarget;

            double motorDriveLeftPower = ENCODER_DRIVE_Kp * motorDriveLeftPercentEncoderError * (1 - motorDriveLeftPercentEncoderError) + ENCODER_DRIVE_POWER_THRESHOLD;
            double motorDriveRightPower = ENCODER_DRIVE_Kp * motorDriveRightPercentEncoderError * (1 - motorDriveRightPercentEncoderError) + ENCODER_DRIVE_POWER_THRESHOLD;

            motorDriveLeftBack.setPower(motorDriveLeftPower);
            motorDriveLeftFront.setPower(motorDriveLeftPower);
            motorDriveRightBack.setPower(motorDriveRightPower);
            motorDriveRightFront.setPower(motorDriveRightPower);

            telemetry.addData("Front Encoders", "(%.2f):(%.2f)", motorDriveLeftFront.getCurrentPosition(),  motorDriveLeftFront.getCurrentPosition());
            telemetry.addData("Back Encoders", "(%.2f):(%.2f)", motorDriveLeftBack.getCurrentPosition(),  motorDriveLeftBack.getCurrentPosition());
            telemetry.addData("Power", "Left (%.2f), Right (%.2f)", motorDriveLeftPower, motorDriveRightPower);
            telemetry.update();
        }

        abortMotion();

        motorDriveLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDriveLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDriveRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDriveRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Encoder Driving COMPLETE");
        telemetry.update();
    }

    public void abortMotion() {
        hasAborted = false;

        motorDriveLeftBack.setPower(0);
        motorDriveLeftFront.setPower(0);
        motorDriveRightBack.setPower(0);
        motorDriveRightBack.setPower(0);
    }
}
