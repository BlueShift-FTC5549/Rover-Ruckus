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

    private DcMotor motorArmRight;
    private DcMotor motorArmLeft;
    private DcMotor motorWinch;

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

        motorArmRight = hardwareMap.get(DcMotor.class, "motorArmRight");
        motorArmLeft = hardwareMap.get(DcMotor.class, "motorArmLeft");
        motorWinch = hardwareMap.get(DcMotor.class, "motorWinch");


        // Since one motor is reversed in relation to the other, we must reverse the motor on the right so positive powers mean forward.
        motorDriveLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorDriveLeftFront.setDirection(DcMotor.Direction.FORWARD);
        motorDriveRightBack.setDirection(DcMotor.Direction.FORWARD);
        motorDriveRightFront.setDirection(DcMotor.Direction.REVERSE);

        motorArmLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorArmRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorWinch.setDirection(DcMotorSimple.Direction.FORWARD);

        motorDriveLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorDriveLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDriveLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDriveRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDriveRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Tell the driver that initialization is complete.
        telemetry.clearAll();
        telemetry.addData("Status", "TeleOP Initialized");
        telemetry.update();
    }

    @Override public void init_loop() { }

    @Override public void start() {
        runtime.reset();
        telemetry.clearAll();
    }

    @Override public void loop() {
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.left_stick_x;
        motorDriveLeftPower = Range.clip(drive + turn, -1.0, 1.0);
        motorDriveRightPower = Range.clip(drive - turn, -1.0, 1.0);

        // Send calculated power to wheels
        motorDriveLeftBack.setPower(motorDriveLeftPower);
        motorDriveLeftFront.setPower(motorDriveLeftPower);
        motorDriveRightBack.setPower(motorDriveRightPower);
        motorDriveRightFront.setPower(motorDriveRightPower);

        double armPower = gamepad1.left_trigger - gamepad1.right_trigger;

        motorArmLeft.setPower(armPower);
        motorArmRight.setPower(armPower);

        motorWinch.setPower(gamepad1.right_stick_y);
    }

    @Override public void stop() {
        motorDriveLeftBack.setPower(0);
        motorDriveLeftFront.setPower(0);
        motorDriveRightBack.setPower(0);
        motorDriveRightFront.setPower(0);
    }
}
