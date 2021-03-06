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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Name Breakdown
 *   Auto - Autonomous Related
 *   Auxiliary - Uses auxiliary motors
 *
 * Necessary Sensors
 *   Encoders:
 *     Lift Motor
 *
 * Function Catalog
 *   liftDrop - Drop the robot from the lander
 *
 * A compilation of functions used during autonomous related to all non-drive motors.
 *
 * @version 0.8
 * @author Gabriel Wong
 */
public class AutoAuxiliary {
    //Robot Motors
    private DcMotor motorLift;
    private DcMotor motorBucket;
    private DcMotor motorSlider;

    //Servos
    private CRServo servoSweeper;

    //Instance Variables
    private boolean hasAborted;
    private boolean verboseLoops;

    //Important Objects to Manipulate
    private Telemetry telemetry;
    private LinearOpMode opMode;
    private ElapsedTime elapsedTime = new ElapsedTime();

    //Arm Deployment Constants
    private static final int SLIDER_HOME_ENCODER_VALUE     = 0;
    private static final int SLIDER_DEPLOYED_ENCODER_VALUE = -3200;
    private static final int BUCKET_HOME_ENCODER_VALUE     = 0;
    private static final int BUCKET_DEPLOYED_ENCODER_VALUE = -435;
    private static final float SLIDER_MOVEMENT_POWER = -0.55f;
    private static final float BUCKET_MOVEMENT_POWER = -0.25f;

    //Lift Constants
    private static final double COUNTS_PER_LIFT_MOTOR_REV = 1680;
    private static final double SHAFT_DIAMETER_INCHES     = 0.23622; //6mm in inches
    private static final double COUNTS_PER_INCH           = COUNTS_PER_LIFT_MOTOR_REV / (SHAFT_DIAMETER_INCHES * 3.1415);
    private static final double DROP_DISTANCE             = 4.69; //Distance needed to drop to ground
    private static final double COUNTS_TO_DROP            = DROP_DISTANCE * COUNTS_PER_INCH;
    private static final float  DROP_POWER                = 1.0f;

    //Misc Constants
    private static final double LOADED_DIRECTION_TEST_POWER = 1.0;
    private static final long   LOADED_DIRECTION_TIME_INTERVAL = 100; //In ms
    private static final float  SERVO_MAX_POWER = 0.8f;
    private static final long   MARKER_EJECTION_TIME_INTERVAL = 1000; //In ms

    /**
     * Create a new instance of an auxiliary autonomous library using all necessary objects and
     * variables.
     *
     * @param opMode The Linear Op Mode that constructs this library
     * @param motorLiftName Name of the lift motor
     * @param verboseLoops Whether or not to use verbose loops
     */
    public AutoAuxiliary(LinearOpMode opMode, String motorLiftName, String motorBucketName, String motorSliderName, String servoSweeperName, boolean verboseLoops) {
        this.motorLift = opMode.hardwareMap.get(DcMotor.class, motorLiftName);
        this.motorBucket = opMode.hardwareMap.get(DcMotor.class, motorBucketName);
        this.motorSlider = opMode.hardwareMap.get(DcMotor.class, motorSliderName);
        this.servoSweeper = opMode.hardwareMap.get(CRServo.class, servoSweeperName);

        this.telemetry = opMode.telemetry;
        this.opMode = opMode;

        this.hasAborted = false;
        this.verboseLoops = verboseLoops;


        motorLift.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBucket.setDirection(DcMotorSimple.Direction.FORWARD);
        motorSlider.setDirection(DcMotorSimple.Direction.FORWARD);

        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBucket.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();
    }

    /**
     * Deploy the arm from the original position to the extended position so that the robot can
     * eject the team marker.
     *
     * @param secondsTimeout The maximum amount of seconds to run the loop for
     */
    public void deployArm(double secondsTimeout) {
        hasAborted = false;

        telemetry.addData("Arm", "Deploying");
        telemetry.update();

        resetEncoderAndRunToPos(motorSlider);
        resetEncoderAndRunToPos(motorBucket);

        motorSlider.setTargetPosition(SLIDER_DEPLOYED_ENCODER_VALUE);
        motorBucket.setTargetPosition(BUCKET_DEPLOYED_ENCODER_VALUE);
        motorSlider.setPower(SLIDER_MOVEMENT_POWER);

        elapsedTime.reset();

        while (elapsedTime.seconds() < secondsTimeout
                && motorSlider.isBusy()
                && !opMode.isStopRequested()
                && !hasAborted) {
            opMode.idle();
        }

        motorSlider.setPower(0);

        motorBucket.setPower(BUCKET_MOVEMENT_POWER);

        while (elapsedTime.seconds() < secondsTimeout
                && motorBucket.isBusy()
                && !opMode.isStopRequested()
                && !hasAborted) {
            opMode.idle();
        }

        motorBucket.setPower(0);


        abortMotion();

        telemetry.addData("Arm", "Deployment Complete");
        telemetry.update();
    }

    /**
     * Retract the arm from a deployed state.
     *
     * @param secondsTimeout The maximum amount of seconds to run the loop for
     */
    public void retractArm(double secondsTimeout) {
        hasAborted = false;

        telemetry.addData("Arm", "Deploying");
        telemetry.update();

        motorSlider.setTargetPosition(SLIDER_HOME_ENCODER_VALUE);
        motorBucket.setTargetPosition(BUCKET_HOME_ENCODER_VALUE);

        elapsedTime.reset();

        motorBucket.setPower(-BUCKET_MOVEMENT_POWER);

        while (elapsedTime.seconds() < secondsTimeout
                && motorBucket.isBusy()
                && !opMode.isStopRequested()
                && !hasAborted) {
            opMode.idle();
        }

        motorBucket.setPower(0);

        motorSlider.setPower(-SLIDER_MOVEMENT_POWER);

        while (elapsedTime.seconds() < secondsTimeout
                && motorSlider.isBusy()
                && !opMode.isStopRequested()
                && !hasAborted) {
            opMode.idle();
        }

        motorSlider.setPower(0);

        abortMotion();

        telemetry.addData("Arm", "Retraction Complete");
        telemetry.update();
    }

    /**
     * Eject the team marker that has been placed inside of the sweeper.
     *
     * @param secondsTimeout The maximum amount of seconds to run the loop for
     */
    public void ejectMarker(double secondsTimeout) {
        hasAborted = false;

        telemetry.addData("Marker Ejection", "Starting");
        telemetry.update();

        elapsedTime.reset();

        while (elapsedTime.seconds() < secondsTimeout
                && elapsedTime.seconds() < MARKER_EJECTION_TIME_INTERVAL
                && !opMode.isStopRequested()
                && !hasAborted) {
            servoSweeper.setPower(SERVO_MAX_POWER);
        }

        abortMotion();

        telemetry.addData("Marker Ejection", "Complete");
    }

    /**
     * Assumes the robot is currently latched onto the lander. Lowers the robot from a latched
     * position onto the game field and then shuts off the lift motor.
     *
     * @param secondsTimeout The maximum amount of seconds to run the loop for
     */
    public void liftDrop(double secondsTimeout) {
        //Set up all variables and do telemetry
        hasAborted = false;

        int loadedDirection = findLoadedDirection(motorLift);
        int dropDirection   = -1 * loadedDirection;
        int previousEncoder = 0;
        int flagCount       = 0; //How many unexpected no-movements has happened
        boolean onGround = false;

        telemetry.addData("Lift Drop", "Starting");
        telemetry.update();

        resetEncoders();
        elapsedTime.reset();

        if (!verboseLoops) {
            while (!onGround
                    && opMode.opModeIsActive()
                    && !opMode.isStopRequested()
                    && !hasAborted
                    && elapsedTime.seconds() <= secondsTimeout) {

                int currentEncoder = motorLift.getCurrentPosition();

                motorLift.setPower(dropDirection * DROP_POWER);

                if (Math.abs(motorLift.getCurrentPosition()) >= COUNTS_TO_DROP) {
                    onGround = true;
                } else if (currentEncoder == previousEncoder) {
                    telemetry.addData("Lift Drop", "WARNING, No movement");
                    telemetry.update();

                    flagCount++;
                }

                if (flagCount > 10) {
                    abortMotion();
                }

                previousEncoder = currentEncoder;
            }
        } else {
            while (!onGround
                    && opMode.opModeIsActive()
                    && !opMode.isStopRequested()
                    && !hasAborted
                    && elapsedTime.seconds() <= secondsTimeout) {

                int currentEncoder = motorLift.getCurrentPosition();

                motorLift.setPower(dropDirection * DROP_POWER);

                if (Math.abs(motorLift.getCurrentPosition()) >= COUNTS_TO_DROP) {
                    onGround = true;
                } else if (currentEncoder == previousEncoder) {
                    telemetry.addData("Lift Drop", "WARNING, No movement");
                    telemetry.update();

                    flagCount++;
                }

                if (flagCount > 10) {
                    abortMotion();
                }

                previousEncoder = currentEncoder;

                telemetry.addData("Current Encoder: ",  currentEncoder);
                telemetry.addData("Target Encoder: ", COUNTS_TO_DROP);
                telemetry.addData("Flag Count", flagCount);
                telemetry.update();
            }
        }

        abortMotion();
    }


    /**
     * Determines the loaded direction of the motor (which power direction requires more power) and
     * returns the sign of that direction (+1 if positive, -1 if negative, 0 if unsure).
     *
     * @param motor The motor for which to find the loaded direction
     * @return The loaded direction of the motor in sign (-1, 1, 0)
     */
    private int findLoadedDirection(DcMotor motor) {
        telemetry.addData("Find Loaded Direction", "Starting for motor: " + getMotorName(motor));
        telemetry.update();

        double negativeEncoderMovement;
        double positiveEncoderMovement;
        int loadedDirection;

        double powerMagnitude = Range.clip(Math.abs(LOADED_DIRECTION_TEST_POWER), 0, 1);

        //Test the negative direction
        resetEncoder(motor);

        motor.setPower(-powerMagnitude);
        opMode.sleep(LOADED_DIRECTION_TIME_INTERVAL);
        motor.setPower(0);

        negativeEncoderMovement = Math.abs(motor.getCurrentPosition());


        //Sleep Interval
        opMode.sleep(LOADED_DIRECTION_TIME_INTERVAL);


        //Test the Positive direction
        resetEncoder(motor);

        motor.setPower(powerMagnitude);
        opMode.sleep(LOADED_DIRECTION_TIME_INTERVAL);
        motor.setPower(0);

        positiveEncoderMovement = Math.abs(motor.getCurrentPosition());


        //Find the value
        if (negativeEncoderMovement < positiveEncoderMovement) {
            loadedDirection = -1;
        } else if (positiveEncoderMovement < negativeEncoderMovement) {
            loadedDirection =  1;
        } else {
            loadedDirection = 0;
        }


        //Telemetry and return the value
        telemetry.addData("Find Loaded Direction", "Finished, loaded direction (" + loadedDirection + ") with -/+ (" + negativeEncoderMovement + ")/(" + positiveEncoderMovement + ")");
        telemetry.update();

        return loadedDirection;
    }

    /**
     * Stop all motion and reset the 'hasAborted' variable for use in while loops. This is used for
     * a sudden halt of the robot and ALL loops that may be running in the autonomous library.
     *
     * Useful for when a stop is requested by the operator.
     */
    private void abortMotion() {
        hasAborted = true;

        motorLift.setPower(0);
        motorBucket.setPower(0);
        motorSlider.setPower(0);

        servoSweeper.setPower(0);

        telemetry.addData("Status", "Auxiliary Motion Aborted");
        telemetry.update();
    }

    /**
     * Stop all motors and reset the current encoder value to zero on each motor. Then, change the
     * motor run modes back to running without the encoder so that we can set the power to them
     * again.
     */
    private void resetEncoders() {
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBucket.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBucket.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Stop a single motor and reset the encoder value to zero. Then change the run mode back so
     * that power can be set.
     *
     * @param motor The motor whose encoder is to be reset
     */
    private void resetEncoder(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Stop a single motor and reset the encoder value to zero. Then change the run mode back so
     * that power can be set. Different from the regular `resetEncoder` method as it sets the motor
     * to RUN_TO_POSITION.
     *
     * @param motor The motor whose encoder is to be reset
     */
    private void resetEncoderAndRunToPos(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * @param motor The motor in question
     * @return the name of a motor in user-readable format.
     */
    private String getMotorName(DcMotor motor) {
        if (motor == motorLift) {
            return "motorLift";
        } else {
            return "unknown";
        }
    }
}
