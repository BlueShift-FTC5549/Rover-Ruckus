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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

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
 *   cubeRemovalAndReturn - Position the Gold Cube in the center of the phone camera view
 *
 * A compilation of useful functions for use during the thirty-second autonomous phase of a FIRST
 * Tech Challenge competition. Created by the programming team of Blue Shift, a function catalog is
 * included above.
 *
 * @version 1.3
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

    //Computer Vision Variables
    private GoldAlignDetector goldAlignDetector; //Detector Object

    //Instance Variables
    private boolean hasAborted;
    private boolean verboseLoops;
    private List<Float> headingStorage;

    //Turning Constants
    private static final float TURN_Kp = 0.75f;
    private static final float TURN_ERROR_ALLOWANCE = (float)5; //In Degrees
    private static final float TURN_POWER_OFFSET_STEP = (float)0.015;

    //Encoder Constants
    private static final double COUNTS_PER_MOTOR_REV    = 1120;    // Andymark Neverest 40
    private static final double DRIVE_GEAR_REDUCTION    = 24.0/32.0;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES   = 4.0;     // For figuring circumference
    private static final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final float  ENCODER_DRIVE_Kp        = 0.85f;
    private static final int    ENCODER_DRIVE_ERROR_ALLOWANCE = 200;
    private static final float  ENCODER_DRIVE_POWER_OFFSET_STEP = (float)0.013;
    private static final int    ENCODER_NO_MOVEMENT_THRESHOLD = 12;

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

        this.telemetry = opMode.telemetry;
        this.opMode = opMode;

        this.hasAborted = false;
        this.verboseLoops = verboseLoops;
        this.headingStorage = new ArrayList<>();


        //Motor Calibration (Direction and Zero Power Behavior)
        motorDriveLeftBack.setDirection(DcMotor.Direction.FORWARD);
        motorDriveLeftFront.setDirection(DcMotor.Direction.FORWARD);
        motorDriveRightBack.setDirection(DcMotor.Direction.REVERSE);
        motorDriveRightFront.setDirection(DcMotor.Direction.REVERSE);

        motorDriveLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //TODO: Decide whether to brake or not
        motorDriveLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDriveRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDriveRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();
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

        int[] previousEncoders = new int[4];
        float thetaInit = imu.getHeading();
        float thetaTarget = (thetaInit + dTheta) % 360;
        float turningPowerOffset = 0.16f;

        telemetry.addData("Turning", "Starting at " + thetaInit + " degrees");
        telemetry.update();

        resetEncoders();
        elapsedTime.reset();

        if (!verboseLoops) {
            while (Math.abs(thetaTarget - imu.getHeading()) % 360.0 > TURN_ERROR_ALLOWANCE
                    && opMode.opModeIsActive()
                    && elapsedTime.seconds() <= secondsTimeout
                    && !hasAborted
                    && !opMode.isStopRequested()) {

                int[] currentEncoders = getEncoderValues();

                double thetaCurrent = (double)imu.getHeading();

                double thetaError = BlueShiftUtil.getDegreeDifference(thetaCurrent, thetaTarget);
                double thetaPercentError = Math.abs(thetaError / dTheta);

                double turnPower = Math.signum(thetaError) * (TURN_Kp * thetaPercentError * (1.0 - thetaPercentError) + turningPowerOffset);

                setTurnPower(turnPower);

                if (!hasAllEncodersMoved(previousEncoders, currentEncoders)) {
                    turningPowerOffset += thetaPercentError * TURN_POWER_OFFSET_STEP; //TODO: Decide whether or not to make offset proportional
                }

                previousEncoders = currentEncoders;
            }
        } else {
            while (Math.abs(thetaTarget - imu.getHeading()) % 360.0 > TURN_ERROR_ALLOWANCE
                    && opMode.opModeIsActive()
                    && elapsedTime.seconds() <= secondsTimeout
                    && !hasAborted
                    && !opMode.isStopRequested()) {

                int[] currentEncoders = getEncoderValues();

                double thetaCurrent = (double)imu.getHeading();

                double thetaError = BlueShiftUtil.getDegreeDifference(thetaCurrent, thetaTarget);
                double thetaPercentError = Math.abs(thetaError / dTheta);

                double turnPower = Math.signum(thetaError) * (TURN_Kp * thetaPercentError * (1.0 - thetaPercentError) + turningPowerOffset);

                setTurnPower(turnPower);

                if (!hasAllEncodersMoved(previousEncoders, currentEncoders)) {
                    turningPowerOffset += thetaPercentError * TURN_POWER_OFFSET_STEP;
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

        telemetry.addData("Turning", "Complete at " + imu.getHeading() + " degrees");
        telemetry.update();
    }

    /**
     * Drive the robot to a certain encoder value on the drive train motors. For use with MECANUM
     * drive trains only. This function strafes instead of drives, meaning it drives left and right
     * laterally instead of forward and backwards as a tank drive would.
     *
     * @param targetDistance The distance in inches to travel
     * @param secondsTimeout The maximum seconds to run the loop
     */
    public void encoderStrafe(double targetDistance, double secondsTimeout) {
        hasAborted = false;

        //Reset encoder values to zero
        resetEncoders();

        //Find desired encoder value and initialize placeholder array
        int[] currentEncoders = getEncoderValues();
        int[] previousEncoders = new int[4];
        int encoderTarget = (int)(targetDistance * COUNTS_PER_INCH);
        double ENCODER_DRIVE_POWER_OFFSET = 0.18;

        //Telemetry Information
        telemetry.addData("Encoder Driving", "Starting at %7d : %7d", motorDriveLeftBack.getCurrentPosition(), motorDriveRightBack.getCurrentPosition());
        telemetry.update();

        //Set the current time to zero
        elapsedTime.reset();

        if (!verboseLoops) {
            while (elapsedTime.seconds() <= secondsTimeout
                    && (Math.abs(BlueShiftUtil.modifiedMeanAbsoluteError(currentEncoders, encoderTarget)) - encoderTarget) > ENCODER_DRIVE_ERROR_ALLOWANCE
                    && opMode.opModeIsActive()
                    && !hasAborted
                    && !opMode.isStopRequested()) {

                currentEncoders = getEncoderValues();

                double encoderMeanAbsoluteError = BlueShiftUtil.modifiedMeanAbsoluteError(currentEncoders, encoderTarget);
                float percentEncoderError = (float)Range.clip(Math.abs((float)encoderMeanAbsoluteError / (float)encoderTarget), 0.0, 1.0);

                double motorPower = Math.signum(encoderMeanAbsoluteError) * (ENCODER_DRIVE_Kp * percentEncoderError * (1 - percentEncoderError) + ENCODER_DRIVE_POWER_OFFSET);

                strafe(motorPower);

                if (!hasAllEncodersMoved(previousEncoders, currentEncoders)) {
                    ENCODER_DRIVE_POWER_OFFSET += ENCODER_DRIVE_POWER_OFFSET_STEP;
                }

                previousEncoders = currentEncoders;
            }
        } else {
            while (elapsedTime.seconds() <= secondsTimeout
                    && Math.abs(BlueShiftUtil.modifiedMeanAbsoluteError(currentEncoders, encoderTarget)) > ENCODER_DRIVE_ERROR_ALLOWANCE
                    && opMode.opModeIsActive()
                    && !hasAborted
                    && !opMode.isStopRequested()) {

                currentEncoders = getEncoderValues();

                double encoderMeanAbsoluteError = BlueShiftUtil.modifiedMeanAbsoluteError(currentEncoders, encoderTarget);
                float percentEncoderError = (float)Range.clip(Math.abs((float)encoderMeanAbsoluteError / (float)encoderTarget), 0.0, 1.0);

                double motorPower = Math.signum(encoderMeanAbsoluteError) * (ENCODER_DRIVE_Kp * percentEncoderError * (1 - percentEncoderError) + ENCODER_DRIVE_POWER_OFFSET);

                strafe(motorPower);

                if (!hasAllEncodersMoved(previousEncoders, currentEncoders)) {
                    ENCODER_DRIVE_POWER_OFFSET += ENCODER_DRIVE_POWER_OFFSET_STEP;
                }

                previousEncoders = currentEncoders;

                telemetry.addData("Front Encoders", "(%7d):(%7d)", motorDriveLeftFront.getCurrentPosition(), motorDriveRightFront.getCurrentPosition());
                telemetry.addData("Back Encoders", "(%7d):(%7d)", motorDriveLeftBack.getCurrentPosition(), motorDriveRightBack.getCurrentPosition());
                telemetry.addLine()
                        .addData("Encoder Target ", encoderTarget + "\n")
                        .addData("Encoder MAE", encoderMeanAbsoluteError + "\n")
                        .addData("Percent Errors", percentEncoderError + "\n")
                        .addData("Power", motorPower + "\n");
                telemetry.update();
            }
        }

        //Stop the robot and terminate any loops running
        abortMotion();

        telemetry.addData("Encoder Driving", "Complete");
        telemetry.update();
    }

    /**
     * Drive the robot to a certain encoder value on the drive train motors.
     *
     * @param targetDistance The distance in inches to travel
     * @param secondsTimeout The maximum seconds to run the loop
     */
    public void encoderDrive(double targetDistance, double secondsTimeout) {
        hasAborted = false;

        //Reset encoder values to zero
        resetEncoders();

        //Find desired encoder value and initialize placeholder array
        int[] previousEncoders = new int[4];
        int encoderTarget = (int)(targetDistance * COUNTS_PER_INCH);
        double ENCODER_DRIVE_POWER_OFFSET = 0.15;

        //Telemetry Information
        telemetry.addData("Encoder Driving", "Starting at %7d : %7d", motorDriveLeftBack.getCurrentPosition(), motorDriveRightBack.getCurrentPosition());
        telemetry.update();

        //Set the current time to zero
        elapsedTime.reset();

        if (!verboseLoops) {
            while (elapsedTime.seconds() <= secondsTimeout
                    && (Math.abs(motorDriveLeftBack.getCurrentPosition() - encoderTarget) > ENCODER_DRIVE_ERROR_ALLOWANCE
                    || Math.abs(motorDriveRightBack.getCurrentPosition() - encoderTarget) > ENCODER_DRIVE_ERROR_ALLOWANCE)
                    && opMode.opModeIsActive()
                    && !hasAborted
                    && !opMode.isStopRequested()) {

                int[] currentEncoders = getEncoderValues();

                int motorDriveLeftEncoderError = encoderTarget - motorDriveLeftBack.getCurrentPosition();
                int motorDriveRightEncoderError = encoderTarget - motorDriveRightBack.getCurrentPosition();
                double motorDriveLeftPercentEncoderError = (double)(motorDriveLeftEncoderError) / (double)encoderTarget;
                double motorDriveRightPercentEncoderError = (double)(motorDriveRightEncoderError) / (double)encoderTarget;

                double motorDriveLeftPower = Math.signum(motorDriveLeftEncoderError) * (ENCODER_DRIVE_Kp * Math.abs(motorDriveLeftPercentEncoderError) * (1 - Math.abs(motorDriveLeftPercentEncoderError)) + ENCODER_DRIVE_POWER_OFFSET);
                double motorDriveRightPower = Math.signum(motorDriveRightEncoderError) * (ENCODER_DRIVE_Kp * Math.abs(motorDriveRightPercentEncoderError) * (1 - Math.abs(motorDriveRightPercentEncoderError)) + ENCODER_DRIVE_POWER_OFFSET);

                setSplitPower(motorDriveLeftPower, motorDriveRightPower);

                if (!hasAllEncodersMoved(previousEncoders, currentEncoders)) {
                    ENCODER_DRIVE_POWER_OFFSET += (motorDriveLeftPercentEncoderError + motorDriveRightPercentEncoderError)/2.0 * ENCODER_DRIVE_POWER_OFFSET_STEP;
                }

                previousEncoders = currentEncoders;
            }
        } else {
            while (elapsedTime.seconds() <= secondsTimeout
                    && (Math.abs(motorDriveLeftBack.getCurrentPosition() - encoderTarget) > ENCODER_DRIVE_ERROR_ALLOWANCE
                    || Math.abs(motorDriveRightBack.getCurrentPosition() - encoderTarget) > ENCODER_DRIVE_ERROR_ALLOWANCE)
                    && opMode.opModeIsActive()
                    && !hasAborted
                    && !opMode.isStopRequested()) {

                int[] currentEncoders = getEncoderValues();

                int motorDriveLeftEncoderError = encoderTarget - motorDriveLeftBack.getCurrentPosition();
                int motorDriveRightEncoderError = encoderTarget - motorDriveRightBack.getCurrentPosition();
                double motorDriveLeftPercentEncoderError = (double)(motorDriveLeftEncoderError) / (double)encoderTarget;
                double motorDriveRightPercentEncoderError = (double)(motorDriveRightEncoderError) / (double)encoderTarget;

                double motorDriveLeftPower = Math.signum(motorDriveLeftEncoderError) * (ENCODER_DRIVE_Kp * Math.abs(motorDriveLeftPercentEncoderError) * (1 - Math.abs(motorDriveLeftPercentEncoderError)) + ENCODER_DRIVE_POWER_OFFSET);
                double motorDriveRightPower = Math.signum(motorDriveRightEncoderError) * (ENCODER_DRIVE_Kp * Math.abs(motorDriveRightPercentEncoderError) * (1 - Math.abs(motorDriveRightPercentEncoderError)) + ENCODER_DRIVE_POWER_OFFSET);

                setSplitPower(motorDriveLeftPower, motorDriveRightPower);

                if (!hasAllEncodersMoved(previousEncoders, currentEncoders)) {
                    ENCODER_DRIVE_POWER_OFFSET += (motorDriveLeftPercentEncoderError + motorDriveRightPercentEncoderError)/2.0 * ENCODER_DRIVE_POWER_OFFSET_STEP;
                }

                previousEncoders = currentEncoders;

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

        telemetry.addData("Encoder Driving", "Complete");
        telemetry.update();
    }

    /**
     * Initialize the gold (cube) alignment detector from DogeCV.
     */
    public void initDogeCV() {
        telemetry.addData("DogeCV", "Initializing");

        // Set up goldAlignDetector
        goldAlignDetector = new GoldAlignDetector(); // Create goldAlignDetector
        goldAlignDetector.init(opMode.hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        goldAlignDetector.useDefaults(); // Set goldAlignDetector to use default settings

        // Optional tuning
        goldAlignDetector.alignSize = 75; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        goldAlignDetector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        goldAlignDetector.downscale = 0.4; // How much to downscale the input frames

        goldAlignDetector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        goldAlignDetector.maxAreaScorer.weight = 0.005; //

        goldAlignDetector.ratioScorer.weight = 5; // Determines the power applied to the motors, the further away it is the faster it turnorer.weight = 5; //
        goldAlignDetector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        goldAlignDetector.enable();

        telemetry.addData("DogeCV", "Initialization Complete");
    }

    /**
     * Assuming the robot's side is aligned with the center debris (position 2). Depending on the
     * given cube position, navigate to the side of that cube and remove it. Then, return back
     * to aligned with the center.
     *
     * @param secondsTimeout The maximum seconds to run the loop for
     * @param cubePosition The position (1, 2, or 3) of the cube
     * @return If successful
     */
    public boolean cubeRemovalAndReturn(double secondsTimeout, int cubePosition) {
        elapsedTime.reset();

        if (elapsedTime.seconds() >= secondsTimeout
                || opMode.isStopRequested()) {
            abortMotion();
            return false;
        }

        if (cubePosition == 1) {
            encoderDrive(18.5, 10);
        } else if (cubePosition == 3) {
            encoderDrive(-17, 10);
        }

        //Move the gold block and back up
        encoderStrafe(-15, 10);
        encoderStrafe(15, 10);

        if (cubePosition == 1) {
            encoderDrive(-12, 10);
        } else if (cubePosition == 3) {
            encoderDrive(22, 10);
        }

        return true;
    }

    public boolean cubeRemovalAndPark(double secondsTimeout, int cubePosition) {
        elapsedTime.reset();

        if (elapsedTime.seconds() >= secondsTimeout
                || opMode.isStopRequested()) {
            abortMotion();
            return false;
        }

        if (cubePosition == 1) {
            encoderDrive(18.5, 10);
        } else if (cubePosition == 3) {
            encoderDrive(-17, 10);
        }

        //Move the gold block and park
        encoderStrafe(-16, 10);
        return true;
    }

    public void disableDogeCV() {
        goldAlignDetector.disable();
    }

    //Utility Functions

    /**
     * Turn to a certain recorded heading.
     *
     * @param headingIndex The index of the stored heading
     * @param secondsTimeout The maximum time to run the `turn` loops for
     */
    public void turnToRecordedHeading(int headingIndex, double secondsTimeout) {
        float dTheta = readHeading(headingIndex) - imu.getHeading();

        turn(dTheta, secondsTimeout);
    }

    /**
     * Turn to a certain recorded heading with an offset.
     *
     * @param headingIndex The index of the stored heading
     * @param headingOffset How much to offset theta by
     * @param secondsTimeout The maximum time to run the `turn` loops for
     */
    public void turnToRecordedHeading(int headingIndex, float headingOffset, double secondsTimeout) {
        float dTheta = (readHeading(headingIndex) - imu.getHeading() + headingOffset) % 360;

        turn(dTheta, secondsTimeout);
    }

    /**
     * Store the current IMU heading.
     *
     * @return The index of the stored heading
     */
    public int recordHeading() {
        if (headingStorage.isEmpty()) {
            headingStorage = Arrays.asList(imu.getHeading());
            return 0;
        }

        headingStorage.add(imu.getHeading());
        return headingStorage.size() - 1;
    }

    /**
     * Read an IMU heading written by the `recordHeading` method.
     *
     * @param index The index of the stored heading
     * @return The stored heading
     */
    public float readHeading(int index) {
        if (headingStorage.isEmpty()
                || index >= headingStorage.size()) {
            return 0;
        }

        return headingStorage.get(index);
    }

    /**
     * Compile all encoder values of all four motors into one array and return that array.
     *
     * @return The array of all four motor encoder values
     */
    private int[] getEncoderValues() {
        return new int[] {
                motorDriveLeftBack.getCurrentPosition(),
                motorDriveLeftFront.getCurrentPosition(),
                motorDriveRightBack.getCurrentPosition(),
                motorDriveRightFront.getCurrentPosition()
        };
    }

    /**
     * Compares two arrays of encoder values and determines whether or not all motors haved moved.
     *
     * @param encoders_1 The first array of encoder values
     * @param encoders_2 The second array of encoder values
     * @return Whether or not all motors have moved
     */
    private boolean hasAllEncodersMoved(int[] encoders_1, int[] encoders_2) {
        if (Math.abs(encoders_1[0] - encoders_2[0]) < ENCODER_NO_MOVEMENT_THRESHOLD
                || Math.abs(encoders_1[1] - encoders_2[1]) < ENCODER_NO_MOVEMENT_THRESHOLD
                || Math.abs(encoders_1[2] - encoders_2[2]) < ENCODER_NO_MOVEMENT_THRESHOLD
                || Math.abs(encoders_1[3] - encoders_2[3]) < ENCODER_NO_MOVEMENT_THRESHOLD) {
            return false;
        }

        return true;
    }

    /**
     * Set the left side to one power and the right side to another
     *
     * @param leftPower The power for the left motors
     * @param rightPower The power for the right motors
     */
    private void setSplitPower(double leftPower, double rightPower) {
        motorDriveLeftBack.setPower(leftPower);
        motorDriveLeftFront.setPower(leftPower);
        motorDriveRightBack.setPower(rightPower);
        motorDriveRightFront.setPower(rightPower);
    }

    /**
     * Set the power of every motor to one power.
     *
     * @param power Desired motor power
     */
    private void setAllPower(double power) {
        motorDriveLeftBack.setPower(power);
        motorDriveLeftFront.setPower(power);
        motorDriveRightBack.setPower(power);
        motorDriveRightFront.setPower(power);
    }

    /**
     * Set the power of every motor to one power except the right side is reversed to turn
     *
     * @param power Desired motor power
     */
    private void setTurnPower(double power) {
        motorDriveLeftBack.setPower(power);
        motorDriveLeftFront.setPower(power);
        motorDriveRightBack.setPower(-power);
        motorDriveRightFront.setPower(-power);
    }

    /**
     * Stop all motion and reset the 'hasAborted' variable for use in while loops. This is used for
     * a sudden halt of the robot and ALL loops that may be running in the autonomous library.
     *
     * Useful for when a stop is requested by the operator.
     */
    private void abortMotion() {
        hasAborted = true;

        setAllPower(0);

        telemetry.addData("Status", "Drive Motion Aborted");
        telemetry.update();
    }

    /**
     * Stop all motors and reset the current encoder value to zero on each motor. Then, change the
     * motor run modes back to running without the encoder so that we can set the power to them
     * again.
     */
    private void resetEncoders() {
        motorDriveLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorDriveLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDriveLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDriveRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDriveRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void strafe(double power) {
        motorDriveLeftFront.setPower(power);
        motorDriveRightBack.setPower(power);

        motorDriveRightFront.setPower(-power);
        motorDriveLeftBack.setPower(-power);
    }

    public boolean isCubeFound() {
        return goldAlignDetector.isFound();
    }
}
