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

import android.media.AudioManager;
import android.media.ToneGenerator;

import com.blueshiftrobotics.ftc.IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * The Blue Shift Autonomous Diagnostics program (BSAD) is designed to test robot and sensor
 * functionality using a series of tests and trials with visual, audio, and in some cases tactile
 * feedback for the user.
 *
 * First, a series of safety tests will be performed that make sure the robot is in the correct
 * state for the general tests to run. Once all checks have succeeded and a human user has given
 * the 'OK' by pressing the start button, the general tests will be performed and audible beeps
 * will sound the beginning and status of each test.
 *
 *
 * Test Catalog
 *
 *  Safety
 *      Robot Off-Ground
 *
 *  General
 *      Encoder Response
 *      Proper Chain-Motor resistance
 *      REV IMU Response
 *
 */
@Autonomous(name="Diagnostics", group="Diagnostics")
public class Auto_Diagnostics extends LinearOpMode {
    //Robot Motors
    private DcMotor motorDriveLeftBack;
    private DcMotor motorDriveLeftFront;
    private DcMotor motorDriveRightBack;
    private DcMotor motorDriveRightFront;

    //Audio objects to play sounds
    private ToneGenerator toneGenerator = new ToneGenerator(AudioManager.STREAM_NOTIFICATION, 100);

    //Important Objects to Manipulate
    private IMU imu;
    private ElapsedTime elapsedTime = new ElapsedTime();

    //Test Constants
    private static final long PRE_TEST_PAUSE = 1000;
    private static final long TEST_TIME_INTERVAL = 500;
    private static final long FLOOR_CONTACT_TIMEOUT = 10000;

    //Robot-Tuned constants
    private static final double NO_MOVEMENT_POWER = 0.08; //What power will the robot not move at while on the ground, but will off the ground
    private static final double ENCODER_RESPONSE_POWER = 0.4;
    private static final int ENCODER_NO_MOVEMENT_TOLERANCE = 100; //Max encoder ticks considered no movement

    //Test Results
    private static boolean encoderResponse;
    private static boolean revIMUResponse;

    private void initialize() {
        setStatus("Initializing");

        this.motorDriveLeftBack   = hardwareMap.get(DcMotor.class, "motorDriveLeftBack");
        this.motorDriveLeftFront  = hardwareMap.get(DcMotor.class, "motorDriveLeftFront");
        this.motorDriveRightBack  = hardwareMap.get(DcMotor.class, "motorDriveRightBack");
        this.motorDriveRightFront = hardwareMap.get(DcMotor.class, "motorDriveRightFront");
        this.imu = new IMU(telemetry, hardwareMap, "imu");

        motorDriveLeftBack.setDirection(DcMotor.Direction.FORWARD);
        motorDriveLeftFront.setDirection(DcMotor.Direction.FORWARD);
        motorDriveRightBack.setDirection(DcMotor.Direction.REVERSE);
        motorDriveRightFront.setDirection(DcMotor.Direction.REVERSE);

        motorDriveLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDriveLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDriveRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDriveRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();

        setStatus("Initialization Complete", "Waiting For Off-Ground Start");
    }

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        //   _________       _____       __
        //  /   _____/____ _/ ____\_____/  |_ ___.__.
        //  \_____  \\__  \\   __\/ __ \   __<   |  |
        //  /        \/ __ \|  | \  ___/|  |  \___  |
        // /_______  (____  /__|  \___  >__|  / ____|
        //         \/     \/          \/      \/
        //

        //Test if the robot is off the ground with a very low motor power
        testFloorContact();

        sleep(PRE_TEST_PAUSE);


        //   ________                                  .__
        //  /  _____/  ____   ____   ________________  |  |
        // /   \  ____/ __ \ /    \_/ __ \_  __ \__  \ |  |
        // \    \_\  \  ___/|   |  \  ___/|  | \// __ \|  |__
        //  \______  /\___  >___|  /\___  >__|  (____  /____/
        //         \/     \/     \/     \/           \/

        //Test encoder and IMU response
        encoderResponse = testEncoderResponse();
        revIMUResponse = testRevIMUResponse();


        sleep(PRE_TEST_PAUSE);
    }

    private boolean testFloorContact() {
        setStatus("Robot Pre-Check", "No Floor Contact");
        boolean onGround = true;

        boolean[] motorStatuses = new boolean[] {false, false, false, false}; //True = passed

        elapsedTime.reset();

        while (onGround
                && opModeIsActive()
                && elapsedTime.seconds() < FLOOR_CONTACT_TIMEOUT) {

            resetEncoders();
            setPowerAll(NO_MOVEMENT_POWER);

            sleep(1000);

            int[] currentEncoders = getEncoderValues();

            for (int i = 0; i < 4; i++) {
                if (Math.abs(currentEncoders[i]) > ENCODER_NO_MOVEMENT_TOLERANCE) {
                    motorStatuses[i] = true;
                }
            }

            if (motorStatuses[0] && motorStatuses[1] && motorStatuses[2] && motorStatuses[3]) {
                onGround = false;

                playGoodTone();
                setStatus("Safety Tests", "No Floor Contact", "PASSED");
            } else {
                playBadTone();
                setStatus("Safety Tests", "No Floor Contact", "FAILING");

                telemetry.addData("Motor Encoder Statuses", "Left Back: "+ motorStatuses[0]
                        + "\nLeft Front: " + motorStatuses[1]
                        + "\nRight Back: " + motorStatuses[2]
                        + "\nRight Front: " + motorStatuses[3]);
            }
        }

        stopMotion();

        return onGround;
    }

    private boolean testEncoderResponse() {
        setStatus("General", "Encoder Response");

        resetEncoders();

        setPowerAll(ENCODER_RESPONSE_POWER);
        sleep(TEST_TIME_INTERVAL);
        stop();

        int[] finalEncoderValues = new int[] {
                motorDriveLeftBack.getCurrentPosition(),
                motorDriveLeftFront.getCurrentPosition(),
                motorDriveRightBack.getCurrentPosition(),
                motorDriveRightFront.getCurrentPosition()
        };

        boolean hasFailedEncoderResponse = false;

        for (int i = 0; i < 4; i++) {
            if (Math.abs(finalEncoderValues[i]) < ENCODER_NO_MOVEMENT_TOLERANCE) {
                hasFailedEncoderResponse = true;
            }
        }

        if (hasFailedEncoderResponse) {
            playBadTone();
            setStatus("General", "Encoder Response", "FAILED");
        } else {
            playGoodTone();
            setStatus("General", "Encoder Response", "PASSED");
        }

        return !hasFailedEncoderResponse;
    }

    private boolean testRevIMUResponse() {
        setStatus("General", "IMU Response");

        boolean hasFailedIMUResponseOrientation = false;
        boolean hasFailedIMUResponseAcceleration = false;
        boolean hasFailedIMUResponse = false;

        Orientation initialOrientation = imu.getOrientation();

        double[] initialHeadings = new double[] {
                initialOrientation.firstAngle,
                initialOrientation.secondAngle,
                initialOrientation.thirdAngle
        };

        setTurnPowerAll(ENCODER_RESPONSE_POWER);

        sleep(TEST_TIME_INTERVAL);

        Acceleration acceleration = imu.getAcceleration();

        double[] accelerations = new double[] {
                acceleration.xAccel,
                acceleration.yAccel,
                acceleration.zAccel
        };

        stopMotion();

        Orientation finalOrientation = imu.getOrientation();

        double[] headings = new double[] {
                finalOrientation.firstAngle,
                finalOrientation.secondAngle,
                finalOrientation.thirdAngle
        };

        if (initialHeadings[0] == headings[0]
                && initialHeadings[1] == headings[1]
                && initialHeadings[2] == initialHeadings[2]) {
            hasFailedIMUResponseOrientation = true;
        }

        if (accelerations[0] == 0.0
                && accelerations[1] == 0.0
                && accelerations[2] == 0.0) {
            hasFailedIMUResponseAcceleration = true;
        }

        hasFailedIMUResponse = !(hasFailedIMUResponseAcceleration && hasFailedIMUResponseOrientation);

        if (hasFailedIMUResponse) {
            playBadTone();
            setStatus("General", "IMU Response", "FAILED");
        } else {
            playGoodTone();
            setStatus("General", "IMU Response", "PASSED");
        }

        return !hasFailedIMUResponse;
    }

    private void setStatus(String status) {
        telemetry.addData("Status", status);
        telemetry.update();
    }
    private void setStatus(String status, String subStatus) {
        telemetry.addLine()
                .addData("Status", status)
                .addData(">", subStatus);
        telemetry.update();
    }
    private void setStatus(String status, String subStatus, String semiStatus) {
        telemetry.addLine()
                .addData("Status", status)
                .addData(">", subStatus)
                .addData(">>", semiStatus);
        telemetry.update();
    }

    private int[] getEncoderValues() {
        return new int[] {
                motorDriveLeftBack.getCurrentPosition(),
                motorDriveLeftFront.getCurrentPosition(),
                motorDriveRightBack.getCurrentPosition(),
                motorDriveRightFront.getCurrentPosition()
        };
    }
    private void resetEncoders() {
        motorDriveLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorDriveLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDriveLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDriveRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDriveRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setPowerAll(double power) {
        motorDriveLeftBack.setPower(power);
        motorDriveLeftFront.setPower(power);
        motorDriveRightBack.setPower(power);
        motorDriveRightFront.setPower(power);
    }
    private void setTurnPowerAll(double power) {
        motorDriveLeftBack.setPower(power);
        motorDriveLeftFront.setPower(power);
        motorDriveRightBack.setPower(-power);
        motorDriveRightFront.setPower(-power);
    }
    private void stopMotion() {
        motorDriveLeftBack.setPower(0);
        motorDriveLeftFront.setPower(0);
        motorDriveRightBack.setPower(0);
        motorDriveRightFront.setPower(0);
    }

    private void playGoodTone() { toneGenerator.startTone(ToneGenerator.TONE_CDMA_KEYPAD_VOLUME_KEY_LITE); }
    private void playBadTone() { toneGenerator.startTone(ToneGenerator.TONE_CDMA_ABBR_ALERT); }
}
