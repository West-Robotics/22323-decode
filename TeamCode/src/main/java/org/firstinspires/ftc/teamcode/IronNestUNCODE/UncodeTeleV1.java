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
            FR.setDirection(DcMotorSimple.Direction.FORWARD);
            BR.setDirection(DcMotorSimple.Direction.REVERSE);
            FL.setDirection(DcMotorSimple.Direction.FORWARD);
            BL.setDirection(DcMotorSimple.Direction.FORWARD);

            FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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


                if(Gamepad1.rightBumper()){
                    FL.setPower(frontLeftPower/6);
                    BL.setPower(backLeftPower/6);
                    FR.setPower(frontRightPower/6);
                    BR.setPower(backRightPower/6);
                }else{FL.setPower(frontLeftPower);
                    BL.setPower(backLeftPower);
                    FR.setPower(frontRightPower);
                    BR.setPower(backRightPower);
                }
                if(Gamepad1.A()||gamepad2.a){
                    liftL.setPosition(0.01);
                    liftR.setPosition(0.99);
                }else{
                        liftL.setPosition(0.22);
                        liftR.setPosition(0.78);
                }
                if(Gamepad1.left_trigger>.2||gamepad2.left_trigger>.2){
                    In.setPower(-1);
                }else if(Gamepad1.leftBumper()||gamepad2.left_bumper){
                    In.setPower(0.25);
                }else{
                    In.setPower(0);
                }



                if(Gamepad1.right_trigger>0.5||gamepad2.right_trigger>0.5){
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
