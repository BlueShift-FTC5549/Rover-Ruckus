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
import com.qualcomm.robotcore.util.ElapsedTime;

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


    //Arm Deployment Constants
    private static final int SLIDER_HOME_ENCODER_VALUE     = -1900;
    private static final int SLIDER_DEPLOYED_ENCODER_VALUE = -3200;
    private static final float SLIDER_MOVEMENT_POWER = -0.6f;

    private static final int BUCKET_HOME_ENCODER_VALUE     = -63;
    private static final int BUCKET_DEPLOYED_ENCODER_VALUE = -420;
    private static final float BUCKET_MOVEMENT_POWER = -0.23f;

    private static final int FLIPPER_HOME_ENCODER_VALUE    = 0;
    private static final int FLIPPER_DEPLOYED_ENCODER_VALUE = 950;
    private static final float FLIPPER_MOVEMENT_POWER = 0.5f;


    //Driving constants and variables
    private static final float SERVO_MAX_POWER = 0.8f;

    private boolean manualControl = false;


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
        motorDriveLeftFront.setDirection(DcMotor.Direction.FORWARD);
        motorDriveRightBack.setDirection(DcMotor.Direction.REVERSE);
        motorDriveRightFront.setDirection(DcMotor.Direction.REVERSE);

        motorDriveLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorDriveLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDriveLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDriveRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDriveRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //Set auxiliary motors parameters
        motorBucket.setDirection(DcMotor.Direction.FORWARD);
        motorFlipper.setDirection(DcMotor.Direction.FORWARD);
        motorLift.setDirection(DcMotor.Direction.FORWARD);
        motorSlider.setDirection(DcMotor.Direction.FORWARD);

        motorBucket.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFlipper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBucket.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFlipper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
        double speed = Math.sqrt(2) * Math.pow(Math.pow(gamepad1.left_stick_x, 4) + Math.pow(gamepad1.left_stick_y, 4), 0.5);
        double angle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);
        double rotation = Math.signum(gamepad1.right_stick_x) * Math.pow(gamepad1.right_stick_x, 2);

        float primaryDiagonalSpeed = (float)(speed * Math.sin(angle - (Math.PI/4.0)));
        float secondaryDiagonalSpeed = (float)(speed * Math.cos(angle - (Math.PI/4.0)));

        motorDriveLeftBack.setPower(secondaryDiagonalSpeed - rotation);
        motorDriveRightFront.setPower(secondaryDiagonalSpeed + rotation);
        motorDriveLeftFront.setPower(primaryDiagonalSpeed - rotation);
        motorDriveRightBack.setPower(primaryDiagonalSpeed + rotation);


        //Lift Code
        if (gamepad1.dpad_up || gamepad2.dpad_up) {
            motorLift.setPower(1.0);
        } else if (gamepad1.dpad_down || gamepad2.dpad_down) {
            motorLift.setPower(-1.0);
        } else {
            motorLift.setPower(0);
        }


        //Flipper Code
        if (gamepad1.left_bumper) {
            extendFlipper();
        } else if (gamepad1.left_trigger > 0) {
            retractFlipper();
        }


        //Servo Code
        if (gamepad1.right_bumper) {
            servoSweeper.setPower(SERVO_MAX_POWER);
        } else if (gamepad1.right_trigger > 0) {
            servoSweeper.setPower(-SERVO_MAX_POWER);
        } else {
            servoSweeper.setPower(0);
        }


        if (gamepad2.a) {
            manualControl = true;
        } else {
            manualControl = false;
        }

        if (manualControl) {
            motorBucket.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFlipper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motorBucket.setPower(gamepad2.right_stick_x);
            motorSlider.setPower(gamepad2.right_stick_y);
            motorFlipper.setPower(gamepad2.left_stick_y);
        } else {
            motorBucket.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFlipper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        //Bucket and Slider Code
        if (gamepad1.a) {
            extendArm();
            extendBucket();
        } else if (gamepad1.b) {
            retractArm();
            retractBucket();
        }
    }

    private void extendArm() {
        if (!manualControl) {
            motorSlider.setTargetPosition(SLIDER_DEPLOYED_ENCODER_VALUE);
            motorSlider.setPower(SLIDER_MOVEMENT_POWER);
        }
    }

    private void retractArm() {
        if (!manualControl) {
            motorSlider.setTargetPosition(SLIDER_HOME_ENCODER_VALUE);
            motorSlider.setPower(-SLIDER_MOVEMENT_POWER);
        }
    }

    private void extendBucket() {
            if (!manualControl) {
                motorBucket.setTargetPosition(BUCKET_DEPLOYED_ENCODER_VALUE);
                motorBucket.setPower(BUCKET_MOVEMENT_POWER);
            }
    }

    private void retractBucket() {
            if (!manualControl) {
                motorBucket.setTargetPosition(BUCKET_HOME_ENCODER_VALUE);
                motorBucket.setPower(-BUCKET_MOVEMENT_POWER);
            }
    }

    private void extendFlipper() {
            if (!manualControl) {
                motorFlipper.setTargetPosition(FLIPPER_DEPLOYED_ENCODER_VALUE);
                motorFlipper.setPower(FLIPPER_MOVEMENT_POWER);
            }
    }

    private void retractFlipper() {
            if (!manualControl) {
                motorFlipper.setTargetPosition(FLIPPER_HOME_ENCODER_VALUE);
                motorFlipper.setPower(-FLIPPER_MOVEMENT_POWER);
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
}
