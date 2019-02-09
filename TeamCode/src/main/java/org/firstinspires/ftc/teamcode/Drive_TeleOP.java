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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOP", group="Main")
public class Drive_TeleOP extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorDriveLeftBack;
    private DcMotor motorDriveLeftFront;
    private DcMotor motorDriveRightBack;
    private DcMotor motorDriveRightFront;

    private DcMotor motorBucket;
    private DcMotor motorFlipper;
    private DcMotor motorLift;
    private DcMotor motorSlider;

    private CRServo servoSweeper;

    private float motorDriveLeftPower, motorDriveRightPower;
    private float servoSweeperPower;

    //Driving constants and variables
    private float drivePowerMultiplier = 1.0f;
    private boolean ifDisableFrontMotors = false;
    private static final float DRIVE_POWER_MULTIPLIER_SLOW = 0.6f;
    private static final float SERVO_MAX_POWER = 0.8f;


    @Override public void init() {
        telemetry.clearAll();
        telemetry.addData("Status", "TeleOP Initialization In Progress");
        telemetry.update();

        // Retrieve the motor objects from the hardware map. These names come from the configuration in the robot controller.
        motorDriveLeftBack = hardwareMap.get(DcMotor.class,   "motorDriveLeftBack");
        motorDriveLeftFront = hardwareMap.get(DcMotor.class,  "motorDriveLeftFront");
        motorDriveRightBack = hardwareMap.get(DcMotor.class,  "motorDriveRightBack");
        motorDriveRightFront = hardwareMap.get(DcMotor.class, "motorDriveRightFront");

        motorBucket = hardwareMap.get(DcMotor.class, "motorBucket");
        motorFlipper = hardwareMap.get(DcMotor.class, "motorFlipper");
        motorLift = hardwareMap.get(DcMotor.class, "motorLift");
        motorSlider = hardwareMap.get(DcMotor.class, "motorSlider");

        servoSweeper = hardwareMap.get(CRServo.class, "servoSweeper");

        // Since one motor is reversed in relation to the other, we must reverse the motor on the right so positive powers mean forward.
        motorDriveLeftBack.setDirection(DcMotor.Direction.FORWARD);
        motorDriveLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorDriveRightBack.setDirection(DcMotor.Direction.REVERSE);
        motorDriveRightFront.setDirection(DcMotor.Direction.FORWARD);

        motorDriveLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorDriveLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDriveLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDriveRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDriveRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //Set auxiliary motors parameters
        motorBucket.setDirection(DcMotor.Direction.FORWARD);
        motorFlipper.setDirection(DcMotor.Direction.REVERSE);
        motorLift.setDirection(DcMotor.Direction.FORWARD);
        motorSlider.setDirection(DcMotor.Direction.FORWARD);

        motorBucket.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFlipper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBucket.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFlipper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorBucket.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFlipper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override public void init_loop() { }

    @Override public void start() {
        runtime.reset();
        telemetry.clearAll();

        telemetry.addData("Status", "Started");
    }

    @Override public void loop() {
        //Driving Code
        float drive = -gamepad1.left_stick_y;
        float turn  =  gamepad1.left_stick_x;

        motorDriveLeftPower = drivePowerMultiplier
                * (float)Range.clip(drive - turn, -1.0, 1.0);
        motorDriveRightPower = drivePowerMultiplier
                * (float)Range.clip(drive + turn, -1.0, 1.0);

        setSplitPower(motorDriveLeftPower, motorDriveRightPower);


        //Box Slider and Bucket Code
        float motorSliderPower = -gamepad1.right_stick_y;

        motorSlider.setPower(motorSliderPower);
        float motorBucketPower;

        if (Math.abs(gamepad1.right_stick_x) > 0.05) {
            motorBucketPower = gamepad1.right_stick_x ;
        } else {
            motorBucketPower = 0;
        }

        if (!gamepad2.a) {
            motorBucket.setPower(motorBucketPower);
        }

        //Lift Code
        if (gamepad1.dpad_up) {
            motorLift.setPower(1.0);
        } else if (gamepad1.dpad_down) {
            motorLift.setPower(-1.0);
        } else {
            motorLift.setPower(0);
        }


        //Sweeper Code
        if (gamepad1.right_trigger > 0) {
            servoSweeperPower = Range.clip(gamepad1.right_trigger, 0, SERVO_MAX_POWER);
        } else if (gamepad1.right_bumper) {
            servoSweeperPower = -SERVO_MAX_POWER;
        } else {
            servoSweeperPower = 0;
        }

        if (!gamepad2.b) {
            servoSweeper.setPower(servoSweeperPower);
        }


        //Flipper Code
        if (gamepad1.left_bumper) {
            motorFlipper.setPower(-0.7);
        } else if (gamepad1.left_trigger > 0) {
            motorFlipper.setPower(gamepad1.left_trigger);
        } else {
            motorFlipper.setPower(0);
        }


        //Modifier Buttons. B = Slow Mode, Y = Disable top two motors
        if (gamepad2.left_bumper) {
            drivePowerMultiplier = DRIVE_POWER_MULTIPLIER_SLOW;

            telemetry.addData("Drive Power Multiplier", drivePowerMultiplier);
        } else {
            drivePowerMultiplier = 1.0f;
        }
    }

    @Override public void stop() {
        motorDriveLeftBack.setPower(0);
        motorDriveLeftFront.setPower(0);
        motorDriveRightBack.setPower(0);
        motorDriveRightFront.setPower(0);

        motorBucket.setPower(0);
        motorFlipper.setPower(0);
        motorLift.setPower(0);
        motorSlider.setPower(0);
    }



    //Miscellaneous
    /**
     * Disable front motor movement
     */
    private void disableFrontMotors() {
        ifDisableFrontMotors = true;

        motorDriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Enable front motor movement
     */
    private void enableFrontMotors() {
        ifDisableFrontMotors = false;

        motorDriveLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDriveRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
}
