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

import com.blueshiftrobotics.ftc.AutoAuxiliary;
import com.blueshiftrobotics.ftc.AutoFourWheelDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Depot Cube ONLY", group="Depot")
public class Auto_Depot_CubeOnly extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private AutoFourWheelDrive autoFourWheelDrive;
    private AutoAuxiliary autoAuxiliary;

    private void initialize() {
        setTelemetryStatus("Initializing");

        autoFourWheelDrive = new AutoFourWheelDrive(this,"motorDriveLeft", "motorDriveRight", "imu", false);
        autoAuxiliary = new AutoAuxiliary(this, "motorLift", false);

        setTelemetryStatus("Initialized");
    }

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        telemetry.clearAll();

        autoFourWheelDrive.initDogeCV();

        //Free the lift hook from the lander
        autoFourWheelDrive.turn(-10, 2.5);
        autoFourWheelDrive.encoderDrive(-0.5, 3);

        //Turn the rest of the angle to be facing away from the lander
        autoFourWheelDrive.turn(-170, 6);

        int centeredHeadingIndex = autoFourWheelDrive.recordHeading();

        //Reverse so the phone camera can scan the block
        autoFourWheelDrive.encoderDrive(-5.5, 6);

        //Find the gold block
        autoFourWheelDrive.cubePositionCenter(10);

        autoFourWheelDrive.stopDogeCV();

        //Move the gold block and back up
        autoFourWheelDrive.encoderDrive(9.5, 10);
    }

    public void setTelemetryStatus(String status) {
        telemetry.addData("Status", status);
        telemetry.update();
    }
}