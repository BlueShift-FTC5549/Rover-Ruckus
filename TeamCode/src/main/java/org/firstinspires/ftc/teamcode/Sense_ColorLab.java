/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * This is an example LinearOpMode that shows how to use
 * the REV Robotics Color-Distance Sensor.
 *
 * It assumes the sensor is configured with the name "sensor_color_distance".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 */
@TeleOp(name = "Color Sensor Lab", group = "Sense") // Comment this out to add to the opmode list
public class Sense_ColorLab extends LinearOpMode {

    private ColorSensor sensorColor;
    private DistanceSensor sensorDistance;

    private boolean retainReading = false;

    private double distanceThreshold = 6.5;
    private double whiteBalanceTreshold = 10;
    private double mean = 0, variation = 0, standardDeviation = 0;
    private double bob = 0;

    private String result = "";

    @Override
    public void runOpMode() {
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor");

        waitForStart();

        telemetry.addData("White Balance Threshhold", whiteBalanceTreshold);

        while (opModeIsActive()) {
            double distance = sensorDistance.getDistance(DistanceUnit.CM);

            if (gamepad1.a) {
                whiteBalanceTreshold++;
            } else if (gamepad1.b) {
                whiteBalanceTreshold--;
            }

            if (gamepad1.x) {
                retainReading = true;
            } else if (gamepad1.y) {
                retainReading = false;
            }

            if(distance < distanceThreshold) {
                int red = sensorColor.red();
                int blue = sensorColor.blue();
                int green = sensorColor.green();

                mean = (red + blue + green) / 3;

                variation = (Math.pow(red - mean, 2) + Math.pow(blue - mean, 2) + Math.pow(green - mean, 2))/3;

                standardDeviation = Math.pow(variation , 0.5);

                bob = mean * standardDeviation/255;

            } else if (retainReading == false) {
                result = "";
            }

            telemetry.addData("Mean", mean);
            telemetry.addData("Variation", variation);
            telemetry.addData("Standard Dev", standardDeviation);
            telemetry.addData("Bob", bob);
            telemetry.addData("White Balance Threshhold", whiteBalanceTreshold);
            telemetry.addData("Result", result);

            telemetry.update();
        }
    }
}
