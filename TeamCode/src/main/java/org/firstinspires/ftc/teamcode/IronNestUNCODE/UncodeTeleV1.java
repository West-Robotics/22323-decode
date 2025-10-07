package org.firstinspires.ftc.teamcode.IronNestUNCODE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.control.Controller;

@TeleOp(name="V1-tele")
public class UncodeTeleV1 extends LinearOpMode{
    @Override
        public void runOpMode() {

            // Initialize the hardware variables. Note that the strings used here must correspond
            // to the names assigned during the robot configuration step on the DS or RC devices.
            DcMotor FL = hardwareMap.get(DcMotor.class, "FrontL");
            DcMotor FR = hardwareMap.get(DcMotor.class, "FrontR");
            DcMotor BR = hardwareMap.get(DcMotor.class, "BackR");
            DcMotor BL = hardwareMap.get(DcMotor.class, "BackL");
            Servo liftL = hardwareMap.get(Servo.class, "LiftL");
            Servo lift2 = hardwareMap.get(Servo.class, "LiftR");
            Controller Gamepad1 = new Controller(gamepad1);
            waitForStart();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                FL.setPower(Gamepad1.left_stick_y+ Gamepad1.left_stick_x+ Gamepad1.right_stick_x);
                FR.setPower(Gamepad1.left_stick_y- Gamepad1.left_stick_x- Gamepad1.right_stick_x);
                BL.setPower(Gamepad1.left_stick_y- Gamepad1.left_stick_x+ Gamepad1.right_stick_x);
                BR.setPower(Gamepad1.left_stick_y+ Gamepad1.left_stick_x- Gamepad1.right_stick_x);
                if(Gamepad1.Y()){
                    liftL.setPosition(1);
                    lift2.setPosition(0);
                }
                if(Gamepad1.A()){
                    liftL.setPosition(0.9);
                    lift2.setPosition(0.1);
                }


            }


    }

}
