package org.firstinspires.ftc.teamcode;

import com.blueshiftrobotics.ftc.AutoFourWheelDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Autobots Rollout", group="Main")
public class Auto_Linear extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private AutoFourWheelDrive autoFourWheelDrive;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        autoFourWheelDrive = new AutoFourWheelDrive(this,"motorDriveLeft", "motorDriveRight", "imu", true);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        runtime.reset();
        telemetry.addData("Status", "Turning...");
        autoFourWheelDrive.encoderDrive(32, 15);
        sleep(3000);
        telemetry.update();
        autoFourWheelDrive.turn(90, 10.0);
        sleep(3000);
        autoFourWheelDrive.turn(-90, 10.0);
        sleep(3000);

        autoFourWheelDrive.encoderDrive(32, 10);
        telemetry.addData("Status", "Finished!");
    }
}
