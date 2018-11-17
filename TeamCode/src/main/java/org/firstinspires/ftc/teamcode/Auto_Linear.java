package org.firstinspires.ftc.teamcode;

import com.blueshiftrobotics.ftc.AutoTwoWheelDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Basic: Linear OpMode", group="Linear Opmode")
public class Auto_Linear extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private AutoTwoWheelDrive autoTwoWheelDrive;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        autoTwoWheelDrive = new AutoTwoWheelDrive(this, hardwareMap, "left", "right", "imu");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        runtime.reset();
        telemetry.addData("Status", "Turning...");
        telemetry.update();
        autoTwoWheelDrive.turn(90);
        sleep(3000);
        autoTwoWheelDrive.turn(180);
        sleep(3000);
        autoTwoWheelDrive.turn(90);
        sleep(3000);

        autoTwoWheelDrive.encoderDrive(3000);
        telemetry.addData("Status", "Finished!");
    }
}
