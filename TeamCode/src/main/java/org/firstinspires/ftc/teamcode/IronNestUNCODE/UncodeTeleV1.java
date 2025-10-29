package org.firstinspires.ftc.teamcode.IronNestUNCODE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.control.Controller;

@TeleOp(name="V2-tele")
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
            Servo liftR = hardwareMap.get(Servo.class, "LiftR");
            DcMotor In = hardwareMap.get(DcMotor.class, "intake");
            DcMotor OutL = hardwareMap.get(DcMotor.class, "outtakeL");
            DcMotor OutR = hardwareMap.get(DcMotor.class, "outtakeR");
            Controller Gamepad1 = new Controller(gamepad1);
            FR.setDirection(DcMotorSimple.Direction.REVERSE);
            BR.setDirection(DcMotorSimple.Direction.REVERSE);
            FL.setDirection(DcMotorSimple.Direction.FORWARD);
            BL.setDirection(DcMotorSimple.Direction.FORWARD);

            waitForStart();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                double y = -Gamepad1.left_stick_y;
                double x = Gamepad1.left_stick_x;
                double rx = Gamepad1.right_stick_x;

                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;

                FL.setPower(frontLeftPower);
                BL.setPower(backLeftPower);
                FR.setPower(frontRightPower);
                BR.setPower(backRightPower);

                if(Gamepad1.Y()){
                    liftL.setPosition(0.01);
                    liftR.setPosition(0.99);
                }
                if(Gamepad1.A()){
                    liftL.setPosition(0.13);
                    liftR.setPosition(0.87);
                }
                if(Gamepad1.leftBumper()){
                    In.setPower(-1);
                }else if(Gamepad1.left_trigger>0.1){
                    In.setPower(Gamepad1.left_trigger);
                }else{
                    In.setPower(0);
                }

                if(Gamepad1.right_trigger>0.5){
                    OutL.setPower(-1);
                    OutR.setPower(1);
                } else{
                    OutL.setPower(0);
                    OutR.setPower(0);
                }
                Gamepad1.update();
            }


    }

}
