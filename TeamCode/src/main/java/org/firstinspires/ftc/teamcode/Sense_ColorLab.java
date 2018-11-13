
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
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
