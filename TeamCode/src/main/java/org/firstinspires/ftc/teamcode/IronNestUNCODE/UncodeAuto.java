package org.firstinspires.ftc.teamcode.IronNestUNCODE;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous  (name = "Scrim auto")
public class UncodeAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        ElapsedTime timer ;
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        DcMotor FL = hardwareMap.get(DcMotor.class, "FrontL");
        DcMotor FR = hardwareMap.get(DcMotor.class, "FrontR");
        DcMotor BR = hardwareMap.get(DcMotor.class, "BackR");
        DcMotor BL = hardwareMap.get(DcMotor.class, "BackL");
        DcMotor OutL = hardwareMap.get(DcMotor.class, "outtakeL");
        DcMotor OutR = hardwareMap.get(DcMotor.class, "outtakeR");
        Servo liftL = hardwareMap.get(Servo.class, "LiftL");
        Servo liftR = hardwareMap.get(Servo.class, "LiftR");
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
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
            if (timer.seconds()<0.7 && !Reached_target_position){
                FL.setPower(0.75); FR.setPower(0.75); BL.setPower(0.75); BR.setPower(0.75);
                sleep(600);
                Reached_target_position = true;
            }else {
                FL.setPower(0); FR.setPower(0); BL.setPower(0); BR.setPower(0);
                sleep(1000);
                    if (timer.seconds()<3 && iteration<3){
                        OutL.setPower(-1); OutR.setPower(1);
                        sleep(15);
                        liftL.setPosition(0.01);
                        liftR.setPosition(0.99);
                        telemetry.addData("Status", "Outtake");
                    }else {
                        liftL.setPosition(0.14);
                        liftR.setPosition(0.86);
                        sleep(3000);
                        timer.reset();
                        iteration += 1;
                        telemetry.addData("Status", "Outtake Comple");
                        if(iteration == 3){break;}
            }
                telemetry.update();
        }
    }
}
}

