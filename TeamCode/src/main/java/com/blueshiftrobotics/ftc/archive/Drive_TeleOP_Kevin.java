package com.blueshiftrobotics.ftc.archive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Drive_TeleOP_Kevin", group="Drive")
public class Drive_TeleOP_Kevin extends OpMode {
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
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.left_stick_x;

        motorDriveLeft.setPower( Range.clip(drive + turn, 0 ,1) );
        motorDriveRight.setPower( Range.clip(drive - turn, 0 ,1) );
    }

    @Override public void stop () {
    }

}
