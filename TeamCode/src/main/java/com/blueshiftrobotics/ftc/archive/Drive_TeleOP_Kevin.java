package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Disabled
@TeleOp(name="Drive_TeleOP_Kevin", group="Drive")
public class Drive_TeleOP_Kevin extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorDriveLeftOne;
    private DcMotor motorDriveLeftTwo;
    private DcMotor motorDriveRightOne;
    private DcMotor motorDriveRightTwo;
    private double motorDriveLeftPower, motorDriveRightPower;
    private double powerMultiplier = 1.0;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialization In Progress");

        // Retrieve the motor objects from the hardware map. These names come from the configuration in the robot controller.
        motorDriveLeftOne = hardwareMap.get(DcMotor.class, "motorDriveLeftOne");
        motorDriveLeftTwo = hardwareMap.get(DcMotor.class, "motorDriveLeftTwo");
        motorDriveRightOne = hardwareMap.get(DcMotor.class, "motorDriveRightOne");
        motorDriveRightTwo = hardwareMap.get(DcMotor.class, "motorDriveRightTwo");

        // Since one motor is reversed in relation to the other, we must reverse the motor on the right so positive powers mean forward.
        motorDriveLeftOne.setDirection(DcMotor.Direction.FORWARD);
        motorDriveLeftTwo.setDirection(DcMotor.Direction.FORWARD);
        motorDriveLeftOne.setDirection(DcMotor.Direction.REVERSE);
        motorDriveRightTwo.setDirection(DcMotor.Direction.REVERSE);

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
        double turn  =  gamepad1.right_stick_x;

        motorDriveLeftPower = Range.clip(drive + turn, -1.0, 1.0) * powerMultiplier;
        motorDriveRightPower = Range.clip(drive - turn, -1.0, 1.0) * powerMultiplier;

        // Send calculated power to wheels
        motorDriveLeftOne.setPower(motorDriveLeftPower);
        motorDriveLeftTwo.setPower(motorDriveLeftPower);
        motorDriveRightOne.setPower(motorDriveRightPower);
        motorDriveRightTwo.setPower(motorDriveRightPower);

    }

    @Override public void stop () {
    }

}
