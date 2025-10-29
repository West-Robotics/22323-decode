package org.firstinspires.ftc.teamcode.IronNestUNCODE;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.control.Controller;

@TeleOp(name="Scrimtele")
public class UncodeScrim extends LinearOpMode{
    @Override
        public void runOpMode() {
            //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            // Initialize the hardware variables. Note that the strings used here must correspond
            // to the names assigned during the robot configuration step on the DS or RC devices.
            DcMotor FL = hardwareMap.get(DcMotor.class, "FrontL");
            DcMotor FR = hardwareMap.get(DcMotor.class, "FrontR");
            DcMotor BR = hardwareMap.get(DcMotor.class, "BackR");
            DcMotor BL = hardwareMap.get(DcMotor.class, "BackL");
            Controller Gamepad1 = new Controller(gamepad1);
            waitForStart();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                FL.setPower(Gamepad1.left_stick_y+ Gamepad1.left_stick_x+ Gamepad1.right_stick_x);
                FR.setPower(Gamepad1.left_stick_y- Gamepad1.left_stick_x- Gamepad1.right_stick_x);
                BL.setPower(Gamepad1.left_stick_y- Gamepad1.left_stick_x+ Gamepad1.right_stick_x);
                BR.setPower(Gamepad1.left_stick_y+ Gamepad1.left_stick_x- Gamepad1.right_stick_x);
            }


    }

}
