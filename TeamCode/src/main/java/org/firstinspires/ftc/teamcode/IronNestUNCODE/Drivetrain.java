package org.firstinspires.ftc.teamcode.IronNestUNCODE;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Drivetrain {
 public void Create_Drivetrain() {
     DcMotorEx FL = hardwareMap.get(DcMotorEx.class, "FrontL");
     DcMotorEx FR = hardwareMap.get(DcMotorEx.class, "FrontR");
     DcMotorEx BR = hardwareMap.get(DcMotorEx.class, "BackR");
     DcMotorEx BL = hardwareMap.get(DcMotorEx.class, "BackL");

     FR.setDirection(DcMotorSimple.Direction.FORWARD);
     BR.setDirection(DcMotorSimple.Direction.REVERSE);
     FL.setDirection(DcMotorSimple.Direction.FORWARD);
     BL.setDirection(DcMotorSimple.Direction.FORWARD);

     FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
 }
 public void ReverseMotor(DcMotor motor){
     motor.setDirection(DcMotorSimple.Direction.REVERSE);
 }

}
