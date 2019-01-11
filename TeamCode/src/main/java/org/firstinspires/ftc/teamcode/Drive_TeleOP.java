package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOP", group="Drive")
public class Drive_TeleOP extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorDriveLeftBack;
    private DcMotor motorDriveLeftFront;
    private DcMotor motorDriveRightBack;
    private DcMotor motorDriveRightFront;
    private DcMotor motorSweeper;

    private double motorDriveLeftPower, motorDriveRightPower;

    @Override public void init() {
        telemetry.clearAll();
        telemetry.addData("Status", "TeleOP Initialization In Progress");
        telemetry.update();

        // Retrieve the motor objects from the hardware map. These names come from the configuration in the robot controller.
        motorDriveLeftBack = hardwareMap.get(DcMotor.class,   "motorDriveLeftBack");
        motorDriveLeftFront = hardwareMap.get(DcMotor.class,  "motorDriveLeftFront");
        motorDriveRightBack = hardwareMap.get(DcMotor.class,  "motorDriveRightBack");
        motorDriveRightFront = hardwareMap.get(DcMotor.class, "motorDriveRightFront");

        motorSweeper = hardwareMap.get(DcMotor.class, "motorSweeper");


        // Since one motor is reversed in relation to the other, we must reverse the motor on the right so positive powers mean forward.
        motorDriveLeftBack.setDirection(DcMotor.Direction.FORWARD);
        motorDriveLeftFront.setDirection(DcMotor.Direction.FORWARD);
        motorDriveRightBack.setDirection(DcMotor.Direction.REVERSE);
        motorDriveRightFront.setDirection(DcMotor.Direction.REVERSE);

        motorSweeper.setDirection(DcMotorSimple.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.clearAll();
        telemetry.addData("Status", "TeleOP Initialized");
        telemetry.update();
    }

    @Override public void init_loop() { }

    @Override public void start() { runtime.reset(); }

    @Override public void loop() {
        if (gamepad1.left_trigger > 0) {
            motorSweeper.setPower(gamepad1.left_trigger);
        } else if (gamepad1.left_bumper){
            motorSweeper.setPower(-1);
        } else {
            motorSweeper.setPower(0);
        }

        double drive = -Math.signum(gamepad1.left_stick_y) * Math.pow(gamepad1.left_stick_y, 2);
        double turn  =  Math.signum(gamepad1.left_stick_x) * Math.pow(gamepad1.left_stick_x, 2);
        motorDriveLeftPower = Range.clip(drive + turn, -1.0, 1.0);
        motorDriveRightPower = Range.clip(drive - turn, -1.0, 1.0);

        // Send calculated power to wheels
        motorDriveLeftBack.setPower(motorDriveLeftPower);
        motorDriveLeftFront.setPower(motorDriveLeftPower);
        motorDriveRightBack.setPower(motorDriveRightPower);
        motorDriveRightFront.setPower(motorDriveRightPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "Left (%.2f), Right (%.2f)", motorDriveLeftPower, motorDriveRightPower);
    }

    @Override public void stop() {

    }
}
