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

package com.blueshiftrobotics.ftc.archive;

import com.blueshiftrobotics.ftc.AutoFourWheelDrive;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Disabled
@Autonomous(name="Operation Check Chain", group="Main")
public class Auto_Kevin extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private AutoFourWheelDrive autoFourWheelDrive;

    private GoldAlignDetector detector;
    private DcMotor motorDriveLeftBack;
    private DcMotor motorDriveLeftFront;
    private DcMotor motorDriveRightBack;
    private DcMotor motorDriveRightFront;

    private static  double POWER = 0.30; //What power will the robot not move at when chain is on
    private static final int ENCODER_NO_MOVEMENT_TOLERANCE = 5; //Max encoder ticks is considered no movement


    @Override
        public void runOpMode() {
        //Init

        autoFourWheelDrive = new AutoFourWheelDrive(this,"motorDriveLeft", "motorDriveRight", "imu", true);


        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");

        motorDriveLeftBack = hardwareMap.get(DcMotor.class, "motorDriveLeftBack");
        motorDriveLeftFront = hardwareMap.get(DcMotor.class, "motorDriveLeftFront");
        motorDriveRightBack = hardwareMap.get(DcMotor.class, "motorDriveRightBack");
        motorDriveRightFront = hardwareMap.get(DcMotor.class, "motorDriveRightFront");

        motorDriveLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorDriveLeftFront.setDirection(DcMotor.Direction.FORWARD);
        motorDriveRightBack.setDirection(DcMotor.Direction.FORWARD);
        motorDriveRightFront.setDirection(DcMotor.Direction.REVERSE);

        // Set up detector
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        detector.enable(); // Start the detector!


        waitForStart();

        //Loop
        check_chains(motorDriveLeftBack, POWER);  // checks if left chain is on
        check_chains(motorDriveRightBack, POWER);    // checks if right chain is on

        }
    private void check_chains(DcMotor motor, double chain_power){   // function sets power to given motor; if chain is on wheels will move slightly, if not there will be no change in encoder
        while (true) {
            int initBackEncoder = motor.getCurrentPosition();   // gets initial encoder value
            motor.setPower(chain_power);
            sleep(500);
            stopMotion();
            int finalBackEncoder = motor.getCurrentPosition();  // gets final encoder value
            if (Math.abs(initBackEncoder - finalBackEncoder) <= ENCODER_NO_MOVEMENT_TOLERANCE) {    // checks if motor has not moved
                telemetry.addData("Chains Is On", motor);
                telemetry.update();
                sleep(1000);
                return;
            } else {
                telemetry.addData("Chains Is Off:",motor);
                telemetry.update();
                sleep(1000);
            }
        }
    }

    /*
    Get off hanger
    Turn 180 degrees
    move back a little bit
    while (doesnt see cube)
        strafe right
        strafe left

    encoder drive to cube
     */
    private void stopMotion() {
        motorDriveLeftBack.setPower(0);
        motorDriveLeftFront.setPower(0);
        motorDriveRightBack.setPower(0);
        motorDriveRightFront.setPower(0);
    }

    public void forward(float power){
        motorDriveLeftFront.setPower(power);
        motorDriveRightFront.setPower(power);
        motorDriveLeftBack.setPower(power);
        motorDriveRightBack.setPower(power);
    }
    public void backward(float power){
        motorDriveLeftFront.setPower(-power);
        motorDriveRightFront.setPower(-power);
        motorDriveLeftBack.setPower(-power);
        motorDriveRightBack.setPower(-power);
    }
    public void turnleft(float power){
        motorDriveLeftFront.setPower(-power);
        motorDriveRightFront.setPower(power);
        motorDriveLeftBack.setPower(-power);
        motorDriveRightBack.setPower(power);
    }
    public void turnright(float power){
        motorDriveLeftFront.setPower(power);
        motorDriveRightFront.setPower(-power);
        motorDriveLeftBack.setPower(power);
        motorDriveRightBack.setPower(-power);
    }
    public void strafeleft(float power){
        motorDriveLeftFront.setPower(-power);
        motorDriveRightFront.setPower(power);
        motorDriveLeftBack.setPower(power);
        motorDriveRightBack.setPower(-power);
    }
    public void straferight(float power){
        motorDriveLeftFront.setPower(power);
        motorDriveRightFront.setPower(-power);
        motorDriveLeftBack.setPower(-power);
        motorDriveRightBack.setPower(power);
    }
}