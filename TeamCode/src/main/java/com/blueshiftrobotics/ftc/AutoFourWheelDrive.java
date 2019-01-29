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

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Name Breakdown
 *   Auto - Autonomous related.
 *   FourWheel - Uses a 4-wheel-drive robot.
 *   Drive - Includes code for moving the robot.
 *
 * Necessary Sensors
 *   IMU (REV Expansion Hub)
 *   Motor Encoders
 *   Positioned Back-Facing Phone Camera
 *
 * Function Catalog
 *   turn               - Use the REV IMU to precisely turn
 *   encoderDrive       - Use the motor encoders to drive a precise distance
 *   initDogeCV         - Activate the DogeCV Libraries for Gold Cube centering
 *   cubePositionCenter - Position the Gold Cube in the center of the phone camera view
 *
 * A compilation of useful functions for use during the thirty-second autonomous phase of a FIRST
 * Tech Challenge competition. Created by the programming team of Blue Shift, a function catalog is
 * included above.
 *
 * @version 1.1
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
    private HardwareMap hardwareMap;

    //Computer Vision Variables
    private GoldAlignDetector goldAlignDetector; //Detector Object
    private static final float goldAlignTurningConstant = .8f;

    //Instance Variables
    private static boolean hasAborted = false;
    private static boolean verboseLoops;


    //Turning Constants
    private static final float TURN_Kp = (float)1;
    private static final float TURN_ERROR_ALLOWANCE = (float)4; //In Degrees
    private static final float TURN_POWER_OFFSET_STEP = (float)0.01;


    //Encoder Constants
    private static final double     COUNTS_PER_MOTOR_REV    = 1120;    // Andymark Neverest 20
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * 3.1415);

    private static final float      ENCODER_DRIVE_Kp        = 0.72f;
    private static final int        ENCODER_DRIVE_ERROR_ALLOWANCE = 20;
    private static final float      ENCODER_DRIVE_POWER_OFFSET_STEP = (float)0.01;

    /**
     * Create a new instance of a four wheel drive library using the current hardware map and names
     * of components of interest.
     *
     * @param opMode The Linear Op Mode that constructs this library
     * @param motorDriveLeftName Name of the left drive motor
     * @param motorDriveRightName Name of the right drive motor
     * @param IMUName Name of the IMU sensor on the REV Expansion Hub
     * @param verboseLoops Whether or not to use verbose loops
     */
    public AutoFourWheelDrive(LinearOpMode opMode, String motorDriveLeftName, String motorDriveRightName, String IMUName, boolean verboseLoops) {
        //Bring in all objects from the OpMode and hardwareMap
        this.motorDriveLeftBack = opMode.hardwareMap.get(DcMotor.class, motorDriveLeftName + "Back");
        this.motorDriveLeftFront = opMode.hardwareMap.get(DcMotor.class, motorDriveLeftName + "Front");
        this.motorDriveRightBack = opMode.hardwareMap.get(DcMotor.class, motorDriveRightName + "Back");
        this.motorDriveRightFront = opMode.hardwareMap.get(DcMotor.class, motorDriveRightName + "Front");
        this.imu = new IMU(opMode.telemetry, opMode.hardwareMap, IMUName);

        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.opMode = opMode;

        this.verboseLoops = verboseLoops;


        //Motor Calibration (Direction and Zero Power Behavior)
        motorDriveLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorDriveLeftFront.setDirection(DcMotor.Direction.FORWARD);
        motorDriveRightBack.setDirection(DcMotor.Direction.FORWARD);
        motorDriveRightFront.setDirection(DcMotor.Direction.REVERSE);

        motorDriveLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //TODO: Decide whether to brake or not
        motorDriveLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDriveRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDriveRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorDriveLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDriveLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDriveRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDriveRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Turn the robot using the REV Expansion Hub IMU using a certain `dTheta` as a parameter. The
     * sign of the dTheta will denote direction, and there is a low-level PID Controller used to
     * control the speed of the motors as the destination is approached at different rates to
     * reduce final error rates from overshoot and undershoot.
     *
     * The turning power P is:
     *              P = s * k * (x) * (1-x)
     * Where s is the desired direction (sign of the error), k is some constant, and x is the
     * percentage error.
     *
     * @param dTheta The angular displacement target
     * @param secondsTimeout The maximum amount of seconds to run the loop for
     */
    public void turn(float dTheta, double secondsTimeout) {
        hasAborted = false;

        float thetaInit = imu.getHeading();
        float thetaTarget = (thetaInit + dTheta) % 360;
        float TURNING_POWER_OFFSET = 0;

        int[] previousEncoders = new int[4];

        telemetry.clearAll();
        telemetry.addData("Status", "Turning");
        telemetry.update();

        resetEncoders();
        elapsedTime.reset();

        if (!verboseLoops) {
            while (Math.abs(thetaTarget - imu.getHeading()) % 360.0 > TURN_ERROR_ALLOWANCE
                    && opMode.opModeIsActive()
                    && elapsedTime.seconds() <= secondsTimeout
                    && !hasAborted) {

                if (opMode.isStopRequested()) {
                    abortMotion();
                }

                int[] currentEncoders = new int[] {
                        motorDriveLeftBack.getCurrentPosition(),
                        motorDriveLeftFront.getCurrentPosition(),
                        motorDriveRightBack.getCurrentPosition(),
                        motorDriveRightFront.getCurrentPosition()
                };

                double thetaCurrent = (double)imu.getHeading();

                double thetaError = BlueShiftUtil.getDegreeDifference(thetaCurrent, thetaTarget);
                double thetaPercentError = Math.abs(thetaError / dTheta);

                double turnPower = Math.signum(thetaError) * (TURN_Kp * thetaPercentError * (1.0 - thetaPercentError) + TURNING_POWER_OFFSET);

                motorDriveLeftBack.setPower(turnPower);
                motorDriveLeftFront.setPower(turnPower);
                motorDriveRightBack.setPower(-turnPower);
                motorDriveRightFront.setPower(-turnPower);

                if (currentEncoders[0] == previousEncoders[0]
                        || currentEncoders[1] == previousEncoders[1]
                        || currentEncoders[2] == previousEncoders[2]
                        || currentEncoders[3] == previousEncoders[3]) {
                    TURNING_POWER_OFFSET += TURN_POWER_OFFSET_STEP;
                }

                previousEncoders = currentEncoders;
            }
        } else {
            while (Math.abs(thetaTarget - imu.getHeading()) % 360.0 > TURN_ERROR_ALLOWANCE
                    && opMode.opModeIsActive()
                    && elapsedTime.seconds() <= secondsTimeout
                    && !hasAborted) {

                if (opMode.isStopRequested()) {
                    abortMotion();
                }

                int[] currentEncoders = new int[] {
                        motorDriveLeftBack.getCurrentPosition(),
                        motorDriveLeftFront.getCurrentPosition(),
                        motorDriveRightBack.getCurrentPosition(),
                        motorDriveRightFront.getCurrentPosition()
                };

                double thetaCurrent = (double)imu.getHeading();

                double thetaError = BlueShiftUtil.getDegreeDifference(thetaCurrent, thetaTarget);
                double thetaPercentError = Math.abs(thetaError / dTheta);

                double turnPower = Math.signum(thetaError) * (TURN_Kp * thetaPercentError * (1.0 - thetaPercentError) + TURNING_POWER_OFFSET);

                motorDriveLeftBack.setPower(turnPower);
                motorDriveLeftFront.setPower(turnPower);
                motorDriveRightBack.setPower(-turnPower);
                motorDriveRightFront.setPower(-turnPower);

                if ((currentEncoders[0] == previousEncoders[0]
                        || currentEncoders[1] == previousEncoders[1]
                        || currentEncoders[2] == previousEncoders[2]
                        || currentEncoders[3] == previousEncoders[3])
                        && thetaPercentError > 0.75) {
                    TURNING_POWER_OFFSET += TURN_POWER_OFFSET_STEP;
                }

                previousEncoders = currentEncoders;

                telemetry.addData("Theta Error", thetaError);
                telemetry.addData("Percent Error", thetaPercentError);
                telemetry.addData("Current Heading", thetaCurrent);
                telemetry.addData("Target Heading", thetaTarget);
                telemetry.addData("Motor Power", turnPower);
                telemetry.update();
            }
        }

        abortMotion();

        telemetry.clearAll();
        telemetry.addData("Status", "Turning Complete");
        telemetry.update();
    }

    /**
     * Drive the robot to a certain encoder value on both drive train motors. The powers of each
     * motor is dependent on the percent distance not traveled yet, so, in essence, it uses a PID
     * controller.
     *
     * @param targetDistance The distance in inches to travel
     * @param secondsTimeout The maximum seconds to run the loop for
     */
    public void encoderDrive(double targetDistance, double secondsTimeout) {
        hasAborted = false;

        //Reset encoder values to zero
        resetEncoders();

        //Find desired encoder value
        int encoderTarget = (int)(targetDistance * COUNTS_PER_INCH);
        double ENCODER_DRIVE_POWER_OFFSET = 0.0;

        int[] previousEncoders = new int[4];

        //Telemetry Information
        telemetry.clearAll();
        telemetry.addData("Status", "Encoder Driving");
        telemetry.addData("Path",  "Starting at %7d : %7d", motorDriveLeftBack.getCurrentPosition(), motorDriveRightBack.getCurrentPosition());
        telemetry.addData("Goal", encoderTarget);
        telemetry.update();

        //Set the current time to zero
        elapsedTime.reset();

        if (!verboseLoops) {
            while (elapsedTime.seconds() <= secondsTimeout
                    && (Math.abs(motorDriveLeftBack.getCurrentPosition() - encoderTarget) > ENCODER_DRIVE_ERROR_ALLOWANCE
                    || Math.abs(motorDriveRightBack.getCurrentPosition() - encoderTarget) > ENCODER_DRIVE_ERROR_ALLOWANCE)
                    && opMode.opModeIsActive()
                    && !hasAborted) {

                if (opMode.isStopRequested()) {
                    abortMotion();
                }

                int[] currentEncoders = new int[] {
                        motorDriveLeftBack.getCurrentPosition(),
                        motorDriveLeftFront.getCurrentPosition(),
                        motorDriveRightBack.getCurrentPosition(),
                        motorDriveRightFront.getCurrentPosition()
                };

                int motorDriveLeftEncoderError = encoderTarget - motorDriveLeftBack.getCurrentPosition();
                int motorDriveRightEncoderError = encoderTarget - motorDriveRightBack.getCurrentPosition();
                double motorDriveLeftPercentEncoderError = (double)(motorDriveLeftEncoderError) / (double)encoderTarget;
                double motorDriveRightPercentEncoderError = (double)(motorDriveRightEncoderError) / (double)encoderTarget;

                double motorDriveLeftPower = Math.signum(motorDriveLeftEncoderError) * (ENCODER_DRIVE_Kp * Math.abs(motorDriveLeftPercentEncoderError) * (1 - Math.abs(motorDriveLeftPercentEncoderError)) + ENCODER_DRIVE_POWER_OFFSET);
                double motorDriveRightPower = Math.signum(motorDriveRightEncoderError) * (ENCODER_DRIVE_Kp * Math.abs(motorDriveRightPercentEncoderError) * (1 - Math.abs(motorDriveRightPercentEncoderError)) + ENCODER_DRIVE_POWER_OFFSET);

                motorDriveLeftBack.setPower(motorDriveLeftPower);
                motorDriveLeftFront.setPower(motorDriveLeftPower);
                motorDriveRightBack.setPower(motorDriveRightPower);
                motorDriveRightFront.setPower(motorDriveRightPower);

                if (currentEncoders[0] == previousEncoders[0]
                        || currentEncoders[1] == previousEncoders[1]
                        || currentEncoders[2] == previousEncoders[2]
                        || currentEncoders[3] == previousEncoders[3]) {
                    ENCODER_DRIVE_POWER_OFFSET += ENCODER_DRIVE_POWER_OFFSET_STEP;
                }

                previousEncoders = currentEncoders;
            }
        } else {
            while (elapsedTime.seconds() <= secondsTimeout
                    && (Math.abs(motorDriveLeftBack.getCurrentPosition() - encoderTarget) > ENCODER_DRIVE_ERROR_ALLOWANCE
                    || Math.abs(motorDriveRightBack.getCurrentPosition() - encoderTarget) > ENCODER_DRIVE_ERROR_ALLOWANCE)
                    && opMode.opModeIsActive()
                    && !hasAborted) {

                if (opMode.isStopRequested()) {
                    abortMotion();
                }

                int[] currentEncoders = new int[] {
                        motorDriveLeftBack.getCurrentPosition(),
                        motorDriveLeftFront.getCurrentPosition(),
                        motorDriveRightBack.getCurrentPosition(),
                        motorDriveRightFront.getCurrentPosition()
                };

                int motorDriveLeftEncoderError = encoderTarget - motorDriveLeftBack.getCurrentPosition();
                int motorDriveRightEncoderError = encoderTarget - motorDriveRightBack.getCurrentPosition();
                double motorDriveLeftPercentEncoderError = (double)(motorDriveLeftEncoderError) / (double)encoderTarget;
                double motorDriveRightPercentEncoderError = (double)(motorDriveRightEncoderError) / (double)encoderTarget;

                double motorDriveLeftPower = Math.signum(motorDriveLeftEncoderError) * (ENCODER_DRIVE_Kp * Math.abs(motorDriveLeftPercentEncoderError) * (1 - Math.abs(motorDriveLeftPercentEncoderError)) + ENCODER_DRIVE_POWER_OFFSET);
                double motorDriveRightPower = Math.signum(motorDriveRightEncoderError) * (ENCODER_DRIVE_Kp * Math.abs(motorDriveRightPercentEncoderError) * (1 - Math.abs(motorDriveRightPercentEncoderError)) + ENCODER_DRIVE_POWER_OFFSET);

                motorDriveLeftBack.setPower(motorDriveLeftPower);
                motorDriveLeftFront.setPower(motorDriveLeftPower);
                motorDriveRightBack.setPower(motorDriveRightPower);
                motorDriveRightFront.setPower(motorDriveRightPower);

                if (currentEncoders[0] == previousEncoders[0]
                        || currentEncoders[1] == previousEncoders[1]
                        || currentEncoders[2] == previousEncoders[2]
                        || currentEncoders[3] == previousEncoders[3]) {
                    ENCODER_DRIVE_POWER_OFFSET += ENCODER_DRIVE_POWER_OFFSET_STEP;
                }

                previousEncoders = currentEncoders;

                telemetry.clearAll();
                telemetry.addData("Boolean checks", (Math.abs(motorDriveLeftBack.getCurrentPosition() - encoderTarget) > ENCODER_DRIVE_ERROR_ALLOWANCE
                        || Math.abs(motorDriveRightBack.getCurrentPosition() - encoderTarget) > ENCODER_DRIVE_ERROR_ALLOWANCE));
                telemetry.addData("Front Encoders", "(%7d):(%7d)", motorDriveLeftFront.getCurrentPosition(), motorDriveRightFront.getCurrentPosition());
                telemetry.addData("Back Encoders", "(%7d):(%7d)", motorDriveLeftBack.getCurrentPosition(), motorDriveRightBack.getCurrentPosition());
                telemetry.addData("Encoder Target", encoderTarget);
                telemetry.addData("Encoder Errors", "(%7d):(%7d)", motorDriveLeftEncoderError, motorDriveRightEncoderError);
                telemetry.addData("Percent Errors", "(%.2f):(%.2f)", motorDriveLeftPercentEncoderError, motorDriveRightPercentEncoderError);
                telemetry.addData("Power", "Left (%.2f), Right (%.2f)", motorDriveLeftPower, motorDriveRightPower);
                telemetry.addData("Timing", "(%.2f) of (%.2f)", elapsedTime.seconds(), secondsTimeout);
                telemetry.update();
            }
        }

        //Stop the robot and terminate any loops running
        abortMotion();

        telemetry.clearAll();
        telemetry.addData("Status", "Encoder Driving COMPLETE");
        telemetry.update();
    }

    /**
     * Set the power of every motor to a single number.
     *
     * @param power Desired motor power
     */
    public void setAllPower(double power) {
        motorDriveLeftBack.setPower(power);
        motorDriveLeftFront.setPower(power);
        motorDriveRightBack.setPower(power);
        motorDriveRightBack.setPower(power);
    }

    /**
     * Stop all motion and reset the 'hasAborted' variable for use in while loops.
     */
    public void abortMotion() {
        hasAborted = true;

        motorDriveLeftBack.setPower(0);
        motorDriveLeftFront.setPower(0);
        motorDriveRightBack.setPower(0);
        motorDriveRightFront.setPower(0);

        telemetry.addData("Status", "Motion Aborted");
        telemetry.update();
    }

    public void resetEncoders() {
        motorDriveLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorDriveLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDriveLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDriveRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDriveRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Initialize the gold (cube) alignment detector from DogeCV.
     */
    public void initDogeCV() {
        telemetry.addData("Status", "Initializing DogeCV");

        // Set up goldAlignDetector
        goldAlignDetector = new GoldAlignDetector(); // Create goldAlignDetector
        goldAlignDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        goldAlignDetector.useDefaults(); // Set goldAlignDetector to use default settings

        // Optional tuning //TODO: Tune this to the robot
        goldAlignDetector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        goldAlignDetector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        goldAlignDetector.downscale = 0.4; // How much to downscale the input frames

        goldAlignDetector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //goldAlignDetector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        goldAlignDetector.maxAreaScorer.weight = 0.005; //

        goldAlignDetector.ratioScorer.weight = 5; // Determines the power applied to the motors, the further away it is the faster it turnorer.weight = 5; //
        goldAlignDetector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        goldAlignDetector.enable(); // Start the goldAlignDetector!
    }

    /**
     * Center the cube so that the robot is facing it. The driving algorithm is the same as the
     * IMU turning algorithm, but the variables are adjusted to fit the situation (centering the
     * gold cube based on camera feedback).
     *
     * @return Whether or not the program was successful.
     */
    public boolean cubePositionCenter(double secondsTimeout) {
        hasAborted = false;

        elapsedTime.reset();

        while (!goldAlignDetector.isFound()
                && opMode.opModeIsActive()
                && elapsedTime.seconds() < secondsTimeout
                && !hasAborted) {

            telemetry.clearAll();
            telemetry.addData("Status", "cubePositionCenter: No Cube Found");
            telemetry.update();

            opMode.sleep(100);
        }

        telemetry.clearAll();
        telemetry.addData("Status", "Centering the Gold Cube");
        telemetry.update();

        double FOVWidth = goldAlignDetector.getInitSize().width;
        double CUBE_CENTERING_POWER_OFFSET = 0.0;

        int[] previousEncoders = new int[4];

        elapsedTime.reset();

        if (!verboseLoops) {
            //Align the robot with the gold cube
            while (!goldAlignDetector.getAligned()
                    && opMode.opModeIsActive()
                    && elapsedTime.seconds() < secondsTimeout
                    && !hasAborted) {

                if (opMode.isStopRequested()) {
                    abortMotion();
                }

                int[] currentEncoders = new int[] {
                        motorDriveLeftBack.getCurrentPosition(),
                        motorDriveLeftFront.getCurrentPosition(),
                        motorDriveRightBack.getCurrentPosition(),
                        motorDriveRightFront.getCurrentPosition()
                };

                double relativeCubePosition = goldAlignDetector.getXPosition() - (FOVWidth / 2.0);
                double alignmentPercentError = Math.abs(Range.clip(Math.abs((float) (relativeCubePosition / (FOVWidth / 2.0))), -1, 1));

                double turnPower = Math.signum(relativeCubePosition) * (goldAlignTurningConstant * alignmentPercentError * (1 - alignmentPercentError) + CUBE_CENTERING_POWER_OFFSET);

                motorDriveLeftBack.setPower(turnPower);
                motorDriveLeftFront.setPower(turnPower);
                motorDriveRightBack.setPower(-turnPower);
                motorDriveRightBack.setPower(-turnPower);

                if (currentEncoders[0] == previousEncoders[0]
                        || currentEncoders[1] == previousEncoders[1]
                        || currentEncoders[2] == previousEncoders[2]
                        || currentEncoders[3] == previousEncoders[3]) {
                    CUBE_CENTERING_POWER_OFFSET += TURN_POWER_OFFSET_STEP;
                }

                previousEncoders = currentEncoders;
            }
        } else {
            while (!goldAlignDetector.getAligned()
                    && opMode.opModeIsActive()
                    && elapsedTime.seconds() < secondsTimeout
                    && !hasAborted) {

                if (opMode.isStopRequested()) {
                    abortMotion();
                }

                int[] currentEncoders = new int[] {
                        motorDriveLeftBack.getCurrentPosition(),
                        motorDriveLeftFront.getCurrentPosition(),
                        motorDriveRightBack.getCurrentPosition(),
                        motorDriveRightFront.getCurrentPosition()
                };

                double relativeCubePosition = goldAlignDetector.getXPosition() - (FOVWidth / 2.0);
                double alignmentPercentError = Math.abs(Range.clip(Math.abs((float) (relativeCubePosition / (FOVWidth / 2.0))), -1, 1));

                double turnPower = Math.signum(relativeCubePosition) * (goldAlignTurningConstant * alignmentPercentError * (1 - alignmentPercentError) + CUBE_CENTERING_POWER_OFFSET);

                motorDriveLeftBack.setPower(turnPower);
                motorDriveLeftFront.setPower(turnPower);
                motorDriveRightBack.setPower(-turnPower);
                motorDriveRightBack.setPower(-turnPower);

                if (currentEncoders[0] == previousEncoders[0]
                        || currentEncoders[1] == previousEncoders[1]
                        || currentEncoders[2] == previousEncoders[2]
                        || currentEncoders[3] == previousEncoders[3]) {
                    CUBE_CENTERING_POWER_OFFSET += TURN_POWER_OFFSET_STEP;
                }

                previousEncoders = currentEncoders;

                telemetry.addData("Cube Position", relativeCubePosition); // Gold X position.
                telemetry.addData("Cube Is Aligned", goldAlignDetector.getAligned());
                telemetry.addData("Alignment Percent Error", alignmentPercentError);
            }
        }

        abortMotion();

        encoderDrive(20, 15);
        opMode.sleep(1000);
        encoderDrive(-20, 15);

        goldAlignDetector.disable();

        telemetry.clearAll();
        telemetry.addData("Status", "Centering Complete");
        telemetry.update();

        return true;
    }
}
