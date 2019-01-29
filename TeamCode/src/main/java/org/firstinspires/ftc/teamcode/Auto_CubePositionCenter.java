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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Cube Position Center", group="Main")
public class Auto_CubePositionCenter extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private AutoFourWheelDrive autoFourWheelDrive;

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        autoFourWheelDrive.cubePositionCenter(15);

        sleep(1000);

        runtime.reset();
        telemetry.addData("Status", "Turning...");
        autoFourWheelDrive.encoderDrive(10, 15);
        sleep(3000);
        telemetry.update();
        autoFourWheelDrive.turn(90, 10.0);
        sleep(3000);
        autoFourWheelDrive.turn(-90, 10.0);
        sleep(3000);

        autoFourWheelDrive.encoderDrive(32, 10);
        telemetry.addData("Status", "Finished!");
    }

    private void initialize() {
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        autoFourWheelDrive = new AutoFourWheelDrive(this,"motorDriveLeft", "motorDriveRight", "imu", true);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        autoFourWheelDrive.initDogeCV();
    }
}
