package com.blueshiftrobotics.ftc;

import android.media.tv.TvInputService;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Drive_TeleOP;

public class AutoTwoWheelDrive {
    private DcMotor motorDriveLeft;
    private DcMotor motorDriveRight;
    private IMU imu;

    private double turnKp = 0.5;
    private float turningErrorAllowance = 2; //In Degrees

    public AutoTwoWheelDrive(HardwareMap hardwareMap, String motorDriveLeftName, String motorDriveRightName, String IMUName) {
        this.motorDriveLeft = motorDriveLeft;
        this.motorDriveRight = motorDriveRight;

        motorDriveLeft = hardwareMap.get(DcMotor.class, motorDriveLeftName);
        motorDriveRight = hardwareMap.get(DcMotor.class, motorDriveRightName);
        imu = new IMU(hardwareMap, IMUName);
    }

    private void turn(float turnAngle) {
        float headingInit;
        float headingFinal;
        float turnErrorPercent;
        float turnErrorInit;

        headingInit = imu.getHeading();
        headingFinal = headingInit + turnAngle;
        turnErrorInit = headingFinal - headingInit;
        turnErrorPercent = turnErrorInit / turnErrorInit;

        //TODO: Use MOD to fix errors when the IMU jumps from 0 to 360 etc
        while (Math.abs(turnErrorPercent) > turningErrorAllowance) {
            turnErrorPercent = (headingFinal - headingInit) / turnErrorInit;

            double turn = turnKp * turnErrorPercent;

            motorDriveLeft.setPower(-turn);
            motorDriveRight.setPower(turn);
        }
    }
}
