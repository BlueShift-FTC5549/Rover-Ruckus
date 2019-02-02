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

import com.blueshiftrobotics.ftc.IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

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

    private IMU imu;

    private float motorDriveLeftPower, motorDriveRightPower;
    private float servoSweeperPower;

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

        imu = new IMU(telemetry, hardwareMap, "imu");

        // Since one motor is reversed in relation to the other, we must reverse the motor on the right so positive powers mean forward.
        motorDriveLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorDriveLeftFront.setDirection(DcMotor.Direction.FORWARD);
        motorDriveRightBack.setDirection(DcMotor.Direction.FORWARD);
        motorDriveRightFront.setDirection(DcMotor.Direction.REVERSE);

        motorDriveLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorDriveLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDriveLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDriveRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDriveRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //Set auxiliary motors to brake
        motorBucket.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFlipper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Tell the driver that initialization is complete.
        telemetry.clearAll();
        telemetry.addData("Status", "TeleOP Initialized");
        telemetry.update();
    }

    @Override public void init_loop() { }

    @Override public void start() {
        runtime.reset();
        telemetry.clearAll();

        imu.initAccelerationLogging();
    }

    @Override public void loop() {
        //
        //Driving Code
        //
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.left_stick_x;
        motorDriveLeftPower = (float)Range.clip(drive + turn, -1.0, 1.0);
        motorDriveRightPower = (float)Range.clip(drive - turn, -1.0, 1.0);

        // Send calculated power to wheels
        motorDriveLeftBack.setPower(motorDriveLeftPower);
        motorDriveLeftFront.setPower(motorDriveLeftPower);
        motorDriveRightBack.setPower(motorDriveRightPower);
        motorDriveRightFront.setPower(motorDriveRightPower);

        if (gamepad1.left_bumper) {
            motorSlider.setPower(1.0);
        } else if (gamepad1.right_bumper) {
            motorSlider.setPower(-1.0);
        } else {
            motorSlider.setPower(0);
        }


        if (gamepad1.left_trigger > 0) {
            motorFlipper.setPower(gamepad1.left_trigger);
        } else if (gamepad1.right_trigger > 0) {
            motorFlipper.setPower(-gamepad1.right_trigger);
        } else {
            motorFlipper.setPower(0);
        }

        if (gamepad1.dpad_up) {
            motorBucket.setPower(1.0);
        } else if (gamepad1.dpad_down) {
            motorBucket.setPower(-1.0);
        } else {
            motorBucket.setPower(0);
        }

        if (gamepad1.a) {
            servoSweeperPower = SERVO_MAX_POWER;
        } else if (gamepad1.b) {
            servoSweeperPower = -SERVO_MAX_POWER;
        } else {
            servoSweeperPower = 0;
        }

        if (servoSweeperPower != servoSweeper.getPower()) {
            servoSweeper.setPower(servoSweeperPower);
        }

        motorLift.setPower(gamepad1.right_stick_y);

        Acceleration accel = imu.getAcceleration();
        Velocity velocity = imu.getVelocity();

        telemetry.addData("Acceleration y:z",
                "\n:" + Math.round(accel.yAccel*100.0)/100.0
                + "\n:" + Math.round(accel.zAccel*100.0)/100.0);

        telemetry.addData("Velocity y:z",
                "\n:" + Math.round(velocity.yVeloc*100.0)/100.0
                + "\n:" + Math.round(velocity.zVeloc*100.0)/100.0);
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
