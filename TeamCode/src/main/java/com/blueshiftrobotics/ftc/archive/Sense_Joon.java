package com.blueshiftrobotics.ftc.archive;

import com.blueshiftrobotics.ftc.IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name="Joon IMU", group="Sense")
public class Sense_Joon extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor left, right;
    IMU imu;
    double Startdegree;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        imu = new IMU(hardwareMap, "imu");
        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");

        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        Startdegree = imu.getHeading();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double currentdegree = imu.getHeading();
        double sum = 0;

        if (currentdegree <= 90.0 && Startdegree >= 270.0 ){
            sum = (360.0 - Startdegree) + currentdegree;
        }
        else if ( Startdegree <= 90.0 && currentdegree >= 270.0){
            sum = (360.0 - currentdegree) + Startdegree;
        }
        else if(Startdegree >= currentdegree){
            sum = Startdegree - currentdegree;
        }
        else if(Startdegree <= currentdegree){
            sum = currentdegree - Startdegree;
        }

        if(sum >= 89.5 && sum <= 90.5) {
            left.setPower(0.0);
            right.setPower(0.0);
        }
        else{
            double powervalue = (90.0 - sum)/180.0;
            if(powervalue < 0.05)
                powervalue = 0.05;
            left.setPower(powervalue);
            right.setPower(-powervalue);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}


