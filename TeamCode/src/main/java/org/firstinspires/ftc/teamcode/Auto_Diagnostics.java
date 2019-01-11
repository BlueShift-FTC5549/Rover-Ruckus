package org.firstinspires.ftc.teamcode;

import android.media.AudioManager;
import android.media.ToneGenerator;

import com.blueshiftrobotics.ftc.AutoFourWheelDrive;
import com.blueshiftrobotics.ftc.IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Test Catalog
 *
 *  Safety
 *      Robot Off-Ground
 *
 *  General
 *      Encoder Response
 *      Arm IMU Response
 *      Arm Not Locked
 *      Proper Chain-Motor resistance
 *      REV IMU Response
 *
 */
@Autonomous(name="Diagnostics", group="Main")
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

    //Robot-Tuned constants
    private static final double NO_MOVEMENT_POWER = 0.1; //What power will the robot not move at while on the ground, but will off the ground
    private static final double ENCODER_RESPONSE_POWER = 0.5;
    private static final int ENCODER_NO_MOVEMENT_TOLERANCE = 5; //Max encoder ticks is considered no movement

    //Test Results
    private static boolean encoderResponse = false;

    @Override
    public void runOpMode() {
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

        motorDriveLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setStatus("Initialization Complete", "Waiting For Off-Ground Start");

        waitForStart();

        sleep(PRE_TEST_PAUSE);


        //   _________       _____       __
        //  /   _____/____ _/ ____\_____/  |_ ___.__.
        //  \_____  \\__  \\   __\/ __ \   __<   |  |
        //  /        \/ __ \|  | \  ___/|  |  \___  |
        // /_______  (____  /__|  \___  >__|  / ____|
        //         \/     \/          \/      \/
        //

        //Test if the robot is off the ground with a very low motor power
        setStatus("Robot Pre-Check", "Floor Contact");
        boolean onGround = true;

        while (onGround && opModeIsActive()) {
            int initBackEncoderAverage = (int) ((motorDriveLeftBack.getCurrentPosition() + motorDriveRightBack.getCurrentPosition()) / 2.0);
            setPowerAll(NO_MOVEMENT_POWER);
            sleep(500);
            stopMotion();
            int finalBackEncoderAverage = (int) ((motorDriveLeftBack.getCurrentPosition() + motorDriveRightBack.getCurrentPosition()) / 2.0);

            if (Math.abs(finalBackEncoderAverage - initBackEncoderAverage) <= ENCODER_NO_MOVEMENT_TOLERANCE) {
                onGround = false;
            } else {
                playBadTone();
                setStatus("Safety Tests", "Floor Contact", "FAILING");
            }
        }

        playGoodTone();
        setStatus("Safety Tests", "Floor Contact", "PASSED");

        sleep(PRE_TEST_PAUSE);


        //   ________                                  .__
        //  /  _____/  ____   ____   ________________  |  |
        // /   \  ____/ __ \ /    \_/ __ \_  __ \__  \ |  |
        // \    \_\  \  ___/|   |  \  ___/|  | \// __ \|  |__
        //  \______  /\___  >___|  /\___  >__|  (____  /____/
        //         \/     \/     \/     \/           \/

        //Test encoder response
        encoderResponse = testEncoderResponse();

        sleep(PRE_TEST_PAUSE);
    }

    private boolean testEncoderResponse() {
        setStatus("General", "Encoder Response");

        int[] initEncoderValues = new int[] {
                motorDriveLeftBack.getCurrentPosition(),
                motorDriveLeftFront.getCurrentPosition(),
                motorDriveRightBack.getCurrentPosition(),
                motorDriveRightFront.getCurrentPosition()
        };

        setPowerAll(ENCODER_RESPONSE_POWER);
        sleep(500);
        stop();

        int[] finalEncoderValues = new int[] {
                motorDriveLeftBack.getCurrentPosition(),
                motorDriveLeftFront.getCurrentPosition(),
                motorDriveRightBack.getCurrentPosition(),
                motorDriveRightFront.getCurrentPosition()
        };

        boolean hasFailedEncoderResponse = false;

        for (int i = 0; i < 4; i++) {
            if (Math.abs(initEncoderValues[i] - finalEncoderValues[i]) > ENCODER_NO_MOVEMENT_TOLERANCE) {
                hasFailedEncoderResponse = true;
            }
        }

        if (hasFailedEncoderResponse) {
            playBadTone();
            setStatus("General", "Encoder Response", "FAILED");
            return false;
        } else {
            playGoodTone();
            setStatus("General", "Encoder Response", "PASSED");
            return true;
        }
    }

    private void setStatus(String status) {
        telemetry.clearAll();
        telemetry.addData("Status", status);
        telemetry.update();
    }

    private void setStatus(String status, String subStatus) {
        telemetry.clearAll();
        telemetry.addLine()
                .addData("Status", status)
                .addData(">", subStatus);
        telemetry.update();
    }

    private void setStatus(String status, String subStatus, String semiStatus) {
        telemetry.clearAll();
        telemetry.addLine()
                .addData("Status", status)
                .addData(">", subStatus)
                .addData(">>", semiStatus);
        telemetry.update();
    }

    private void setPowerAll(double power) {
        motorDriveLeftBack.setPower(power);
        motorDriveLeftFront.setPower(power);
        motorDriveRightBack.setPower(power);
        motorDriveRightFront.setPower(power);
    }

    private void stopMotion() {
        motorDriveLeftBack.setPower(0);
        motorDriveLeftFront.setPower(0);
        motorDriveRightBack.setPower(0);
        motorDriveRightFront.setPower(0);
    }

    private void playGoodTone() {
        toneGenerator.startTone(ToneGenerator.TONE_CDMA_KEYPAD_VOLUME_KEY_LITE);
    }

    private void playBadTone() {
        toneGenerator.startTone(ToneGenerator.TONE_CDMA_KEYPAD_VOLUME_KEY_LITE);
    }
}
