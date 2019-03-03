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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Depot All", group="Depot")
public class Auto_Depot_All extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private AutoFourWheelDrive autoFourWheelDrive;
    private AutoAuxiliary autoAuxiliary;
    private CRServo servoSweeper;

    private void initialize() {
        setTelemetryStatus("Initializing");

        autoFourWheelDrive = new AutoFourWheelDrive(this,"motorDriveLeft", "motorDriveRight", "imu", true);
        autoAuxiliary = new AutoAuxiliary(this, "motorLift", "motorBucket", "motorSlider", "servoSweeper", false);

        setTelemetryStatus("Initialized");
    }

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        autoFourWheelDrive.initDogeCV();

        telemetry.clearAll();
        int cubePosition = 0; //1, 2, or 3. Corresponds left to right from the perspective of the lander

        autoAuxiliary.liftDrop(15);
        autoFourWheelDrive.encoderStrafe(-5.0, 10); //Free the lift hook from the lander
        sleep(100);
        autoFourWheelDrive.encoderDrive(5, 5);
        sleep(100);
        autoFourWheelDrive.turn(64, 10); //Turn the an angle <90 to get cube position 3 in view

        sleep(250); //Sleep for a small interval and check if the cube is in position 3
        if (autoFourWheelDrive.isCubeFound()) { cubePosition = 3; }

        sleep(750);

        autoFourWheelDrive.turn(22, 10); //Turn the rest of the angle to complete 90

        //Finalize cube position and tell driver
        if (autoFourWheelDrive.isCubeFound() && cubePosition != 3) {
            cubePosition = 2;
        } else if (cubePosition == 0){
            cubePosition = 1;
        }
        setTelemetryStatus("Cube Position: " + cubePosition);

        sleep(750);

        autoFourWheelDrive.encoderStrafe(-14,10); //Get closer to the cube

        //Remove the gold cube
        setTelemetryStatus("Removing Gold Cube and Returning");
        autoFourWheelDrive.cubeRemovalAndReturn(10, cubePosition);

        setTelemetryStatus("Aligned, Turning to Deploy Arm");
        autoFourWheelDrive.turn(-90, 10);
        autoAuxiliary.deployArm(7);

        setTelemetryStatus("Ejecting Marker");
        autoAuxiliary.ejectMarker(3);

        autoAuxiliary.retractArm(7);

        autoFourWheelDrive.disableDogeCV();

        //autoFourWheelDrive.encoderStrafe(48, 15);
    }

    public void setTelemetryStatus(String status) {
        telemetry.addData("Status", status);
        telemetry.update();
    }
}