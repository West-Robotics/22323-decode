package org.firstinspires.ftc.teamcode.IronNestUNCODE;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous  (name = "The auto blue")
public class UncodeAutoBlue extends LinearOpMode{
    @Override
    public void runOpMode() {
        ElapsedTime timer ;
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        DcMotorEx FL = hardwareMap.get(DcMotorEx.class, "FrontL");
        DcMotorEx FR = hardwareMap.get(DcMotorEx.class, "FrontR");
        DcMotorEx BR = hardwareMap.get(DcMotorEx.class, "BackR");
        DcMotorEx BL = hardwareMap.get(DcMotorEx.class, "BackL");
        DcMotorEx OutL = hardwareMap.get(DcMotorEx.class, "outtakeL");
        DcMotorEx OutR = hardwareMap.get(DcMotorEx.class, "outtakeR");
        DcMotorEx In = hardwareMap.get(DcMotorEx.class, "intake");
        Servo liftL = hardwareMap.get(Servo.class, "LiftL");
        Servo liftR = hardwareMap.get(Servo.class, "LiftR");
        FR.setDirection(DcMotorSimple.Direction.FORWARD);

        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        int iteration  = 0 ;
        boolean Reached_target_position = false;
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        timer = new ElapsedTime();
        waitForStart();
        timer.reset();
        while (opModeIsActive()){
            if (timer.seconds()<0.5 && !Reached_target_position){
                FL.setPower(0.375); FR.setPower(0.375); BL.setPower(0.375); BR.setPower(0.375);
                sleep(1800);
                Reached_target_position = true;
                FL.setPower(0); FR.setPower(0); BL.setPower(0); BR.setPower(0);
            }else {
                FL.setPower(0); FR.setPower(0); BL.setPower(0); BR.setPower(0);
                sleep(1000);
                if (timer.seconds()<3 && iteration<3){
                    OutL.setPower(-1); OutR.setPower(1);
                    In.setPower(-1);
                    sleep(15);
                    liftL.setPosition(0.01);
                    liftR.setPosition(0.99);
                    telemetry.addData("Status", "Outtake");
                }else {
                    liftL.setPosition(0.22);
                    liftR.setPosition(0.78);
                    sleep(1000);
                    timer.reset();
                    iteration += 1;
                    telemetry.addData("Status", "Outtake Complete");
                    if(iteration == 3){
                        FL.setPower(0.75); FR.setPower(-0.75); BL.setPower(-0.75); BR.setPower(0.75);
                        sleep(750);
                        break;}
                }
                telemetry.update();
            }
        }
     }
}

