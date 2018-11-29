package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.blueshiftrobotics.ftc.AutoTwoWheelDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Encoder drive function testing.
 *
 * @author Joon Kang
 */
@TeleOp(name="Encoder Test Joon", group="Test")
public class Encoder_Test_Joon extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorDriveLeft;
    private DcMotor motorDriveRight;

    int Counter = 0;

    @Override public void init() {
        telemetry.addData("Status", "Initialization In Progress");

        // Retrieve the motor objects from the hardware map. These names come from the configuration in the robot controller.
        motorDriveLeft = hardwareMap.get(DcMotor.class, "motorDriveLeft");
        motorDriveRight = hardwareMap.get(DcMotor.class, "motorDriveRight");

        // Since one motor is reversed in relation to the other, we must reverse the motor on the right so positive powers mean forward.
        motorDriveLeft.setDirection(DcMotor.Direction.FORWARD);
        motorDriveRight.setDirection(DcMotor.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    @Override public void loop() {


        int distance = 30;

        if (Counter == 1) {
            motorDriveLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorDriveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motorDriveRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorDriveRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motorDriveLeft.setTargetPosition((distance/4)*1440);
            motorDriveRight.setTargetPosition((distance/4)*1440);

            motorDriveLeft.setPower(1.0);
            motorDriveRight.setPower(1.0);

            System.out.println("Is Busy:" + motorDriveLeft.isBusy());

            while (motorDriveLeft.isBusy()) {
                System.out.println("Pos: " + motorDriveLeft.getCurrentPosition());
                idle();
            }

            motorDriveLeft.setPower(0.0);
            motorDriveRight.setPower(0.0);

            Counter = Counter + 1;
        }

        if (Counter == 3) {
            motorDriveLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorDriveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motorDriveRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorDriveRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motorDriveLeft.setTargetPosition(-(distance/4)*1440);
            motorDriveRight.setTargetPosition(-(distance/4)*1440);

            motorDriveLeft.setPower(-1.0);
            motorDriveRight.setPower(-1.0);

            while (motorDriveLeft.isBusy()) {
                idle();
            }

            motorDriveLeft.setPower(0.0);
            motorDriveRight.setPower(0.0);

            Counter = Counter + 1;
        }

        if (gamepad1.a) {
            Counter = Counter + 1;
        }
    }

    private void idle() {
    }
}
