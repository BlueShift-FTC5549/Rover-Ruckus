/* Copyright (c) 2018 Blue Shift Robotics
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.blueshiftrobotics.ftc.AutoFourWheelDrive;
import com.blueshiftrobotics.ftc.IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * An autonomous OpMode that starts hanging from the crater side of the lander. It incorporates the
 * autonomous library {@link AutoFourWheelDrive} to perform many of the following functions:
 *
 *  1. Drop from the lander
 *  2. Turn 180 degrees
 *  3. Reverse to get the gold cube and both silver spheres in frame of the phone camera
 *  4. Center the gold cube in front of the robot and drive forward to knock it away
 *  5.
 */
@Autonomous(name="Crater Drop", group="Main")
public class Auto_Crater extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private AutoFourWheelDrive autoFourWheelDrive;

    private IMU imu;

    private void initialize() {
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        autoFourWheelDrive = new AutoFourWheelDrive(this,"motorDriveLeft", "motorDriveRight", "imu", true);

        imu = new IMU(telemetry, hardwareMap, "imu");

        autoFourWheelDrive.initDogeCV();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
    }

    @Override
    public void runOpMode() {
        initialize();

        //Lower the robot from the lander
        boolean onGround = false;

        while (!onGround
                && opModeIsActive()
                && !isStopRequested()) {

            onGround = true;

            double verticalAcceleration = imu.getAcceleration().zAccel;
        }

        autoFourWheelDrive.turn(180, 10);

        autoFourWheelDrive.encoderDrive(-7, 10);

        autoFourWheelDrive.cubePositionCenter(20);

        autoFourWheelDrive.encoderDrive(17, 10);
        autoFourWheelDrive.encoderDrive(-14, 10);
    }
}
