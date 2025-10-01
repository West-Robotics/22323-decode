package org.firstinspires.ftc.teamcode.IronNestUNCODE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
@Autonomous  (name = "Scrim auto")
public class UncodeAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        ElapsedTime timer ;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        DcMotor FL = hardwareMap.get(DcMotor.class, "FrontL");
        DcMotor FR = hardwareMap.get(DcMotor.class, "FrontR");
        DcMotor BR = hardwareMap.get(DcMotor.class, "BackR");
        DcMotor BL = hardwareMap.get(DcMotor.class, "BackL");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        timer = new ElapsedTime();
        waitForStart();

        while (opModeIsActive()){
            timer.reset();
            if (timer.seconds()<1.5){
                FL.setPower(0.75); FR.setPower(-0.75); BL.setPower(0.75); BR.setPower(-0.75);
            }else {
                FL.setPower(0); FR.setPower(0); BL.setPower(0); BR.setPower(0);
            }
            telemetry.update();
        }
    }
}
