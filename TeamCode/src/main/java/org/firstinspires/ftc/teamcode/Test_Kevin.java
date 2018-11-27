package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Test_Kevin", group="Drive")
public class Test_Kevin extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorDriveLeft;
    private DcMotor motorDriveRight;

    @Override
    public void init() {
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

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        if (gamepad1.right_trigger > 0) {
            motorDriveLeft.setPower(1);
            motorDriveRight.setPower(1);
        }
        else if (gamepad1.left_trigger < 0) {
            motorDriveLeft.setPower(-1);
            motorDriveRight.setPower(-1);
        }
        else if(gamepad1.left_stick_x > 0){
            motorDriveLeft.setPower(-gamepad1.left_stick_x);
            motorDriveRight.setPower(gamepad1.left_stick_x);
        }
        else if (gamepad1.left_stick_x < 0) {
            motorDriveLeft.setPower(gamepad1.left_stick_x);
            motorDriveRight.setPower(-gamepad1.left_stick_x);
        }
        else if (gamepad1.a) {
            motorDriveLeft.setPower(1);
            motorDriveRight.setPower(-1);
            try {
                Thread.sleep(3000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            motorDriveRight.setPower(0);
            motorDriveLeft.setPower(0);
        }
        else {
            motorDriveRight.setPower(0);
            motorDriveLeft.setPower(0);
        }
    }

    @Override public void stop () {
    }

}
