package org.firstinspires.ftc.teamcode;

import com.blueshiftrobotics.ftc.AutoFourWheelDrive;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Sky Net", group="Main")
public class Auto_Kevin extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private AutoFourWheelDrive autoFourWheelDrive;

    private GoldAlignDetector detector;
    private DcMotor motorDriveLeftBack;
    private DcMotor motorDriveLeftFront;
    private DcMotor motorDriveRightBack;
    private DcMotor motorDriveRightFront;
    double x_pos;
    double power;
    double chain_power;

    private static  double NO_MOVEMENT_POWER = 0.01; //What power will the robot not move at while on the ground, but will off the ground
    private static final int ENCODER_NO_MOVEMENT_TOLERANCE = 5; //Max encoder ticks is considered no movement
    private boolean chainON = true;

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
        while (true) {
            while (chainON && opModeIsActive()) {
                int initBackEncoderAverage = (int) ((motorDriveLeftBack.getCurrentPosition() + motorDriveRightBack.getCurrentPosition()) / 2.0);
                setPowerAll(NO_MOVEMENT_POWER);
                sleep(500);
                stopMotion();
                int finalBackEncoderAverage = (int) ((motorDriveLeftBack.getCurrentPosition() + motorDriveRightBack.getCurrentPosition()) / 2.0);
                if (Math.abs(finalBackEncoderAverage - initBackEncoderAverage) <= ENCODER_NO_MOVEMENT_TOLERANCE) {
                    telemetry.addData("Chains Status:","ON",NO_MOVEMENT_POWER);
                    telemetry.update();
                    chainON = false;
                } else {
                    telemetry.addData("Chains Status:","OFF");
                    telemetry.update();
                    NO_MOVEMENT_POWER += 0.01;
                    sleep(50);
                }
            }
            /* x_pos = detector.getXPosition();

            if (x_pos >= 300 && x_pos <= 340) {
                autoFourWheelDrive.encoderDrive(20, 15);
                telemetry.addData("Using encoder" ,"True");

            } else if (x_pos < 310) {
                power =(1.0-(x_pos / 310))*0.5;
                if (power < 0.3) power = 0.2;
                motorDriveLeftFront.setPower(0);
                motorDriveLeftBack.setPower(0);
                motorDriveRightBack.setPower(power);
                motorDriveRightFront.setPower(power);
                telemetry.addData("Turing Right Wheels", "True");

            } else if (x_pos > 330) {
                power = (1.0-((640 - x_pos) / 310))*0.5;
                if (power < 0.3) power = 0.2;
                motorDriveRightBack.setPower(0);
                motorDriveRightFront.setPower(0);
                motorDriveLeftBack.setPower(power);
                motorDriveLeftFront.setPower(power);
                telemetry.addData("Turning Left Wheels", "True");
            }
            telemetry.addData("IsAligned", detector.getAligned()); // Is the bot aligned with the gold mineral?
            telemetry.addData("X Pos", detector.getXPosition()); // Gold X position.
            telemetry.addData("Power ", 1.0 - power);
            telemetry.addData("Using encoder","False");
            telemetry.addData("Turning Right Wheels","False");
            telemetry.addData("Turning Left Wheels","False");
            telemetry.update();
            */
        }
    }

    private void setPowerAll(double chain_power) {
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


}
