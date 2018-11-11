package org.firstinspires.ftc.teamcode;

import com.blueshiftrobotics.ftc.IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name="TeleOP", group="Drive")
public class Drive_TeleOP extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorDriveLeft;
    private DcMotor motorDriveRight;

    private double motorDriveLeftPower, motorDriveRightPower;
    private double powerMultiplier = 1.0;

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

    @Override public void init_loop() { }

    @Override public void start() { runtime.reset(); }

    @Override public void loop() {
        // The power multiplier scales down non-linearly as the left trigger is pushed more.
        // Scaling follows the function (2/3)*(1-x)^2+(1/3) where x is the trigger value.
        powerMultiplier = (2/3)*Math.pow(1 - gamepad1.left_trigger, 2) + (1/3);

        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        motorDriveLeftPower = Range.clip(drive + turn, -1.0, 1.0) * powerMultiplier;
        motorDriveRightPower = Range.clip(drive - turn, -1.0, 1.0) * powerMultiplier;

        // Send calculated power to wheels
        motorDriveLeft.setPower(motorDriveLeftPower);
        motorDriveRight.setPower(motorDriveRightPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "Left (%.2f), Right (%.2f)", motorDriveLeftPower, motorDriveRightPower);
        telemetry.addData("Modifiers", "Power Multiplier (%.2f)", powerMultiplier);
    }

    @Override public void stop() {

    }
}