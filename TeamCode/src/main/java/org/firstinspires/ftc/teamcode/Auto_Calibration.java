package org.firstinspires.ftc.teamcode;

import android.media.AudioManager;
import android.media.ToneGenerator;

import com.blueshiftrobotics.ftc.IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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
@Autonomous(name="Calibration", group="Diagnostics")
public class Auto_Calibration extends LinearOpMode {
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
    private static final long PAUSE_INTERVAL = 1000;
    private static final double MOTOR_POWER_STEP = 0.01;
    private static final int ENCODER_MOVEMENT_THRESHOLD = 10;

    //Constants to Find
    private static double minimumMovementPower;

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

        playStatusTone();
        setStatus("Starting", "Prepare for Movement in " + PAUSE_INTERVAL + "ms");
        sleep(PAUSE_INTERVAL);


        minimumMovementPower = findMinimumMovementPower();
        sleep(10000);
    }

    private double findMinimumMovementPower() {
        playStatusTone();
        setStatus("Running Test", "Minimum Movement Power");

        resetEncoders();

        boolean robotHasMoved = false;
        double motorPower = 0.0;

        while (!robotHasMoved && opModeIsActive() && motorPower < 1.0) {
            motorPower += MOTOR_POWER_STEP;
            setPower(motorPower);

            int[] encoderValues = new int[] {
                    motorDriveLeftBack.getCurrentPosition(),
                    motorDriveLeftFront.getCurrentPosition(),
                    motorDriveRightBack.getCurrentPosition(),
                    motorDriveRightFront.getCurrentPosition()
            };

            robotHasMoved = (encoderValues[0] > ENCODER_MOVEMENT_THRESHOLD
                    && encoderValues[1] > ENCODER_MOVEMENT_THRESHOLD
                    && encoderValues[2] > ENCODER_MOVEMENT_THRESHOLD
                    && encoderValues[3] > ENCODER_MOVEMENT_THRESHOLD);
        }

        playPositiveTone();
        setStatus("Test Complete", "Minimum Movement Power", "Value: " + motorPower);
        return motorPower;
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

    private void setPower(double power) {
        motorDriveLeftBack.setPower(power);
        motorDriveLeftFront.setPower(power);
        motorDriveRightBack.setPower(power);
        motorDriveRightFront.setPower(power);
    }

    private void playPositiveTone() { toneGenerator.startTone(ToneGenerator.TONE_CDMA_KEYPAD_VOLUME_KEY_LITE); }

    private void playStatusTone() { toneGenerator.startTone(ToneGenerator.TONE_CDMA_ABBR_ALERT); }
}
