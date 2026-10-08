package org.firstinspires.ftc.teamcode.IronNestBEE

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap

class KotlnDrivetrain( hardwareMap: HardwareMap){
    /**
     * Initialize all motors from the hardware map.
     */
    init {
        val frontLeftMotor = hardwareMap.get("FrontL") as DcMotorEx
        val frontRightMotor = hardwareMap.get("FrontR") as DcMotorEx
        val backLeftMotor = hardwareMap.get("BackL") as DcMotorEx
        val backRightMotor = hardwareMap.get("BackR") as DcMotorEx

        frontRightMotor.direction = DcMotorSimple.Direction.FORWARD
        frontLeftMotor.direction = DcMotorSimple.Direction.FORWARD
        backRightMotor.direction = DcMotorSimple.Direction.REVERSE
        backLeftMotor.direction = DcMotorSimple.Direction.REVERSE

        frontRightMotor.zeroPowerBehavior= DcMotor.ZeroPowerBehavior.BRAKE
        frontLeftMotor.zeroPowerBehavior= DcMotor.ZeroPowerBehavior.BRAKE
        backRightMotor.zeroPowerBehavior= DcMotor.ZeroPowerBehavior.BRAKE
        backLeftMotor.zeroPowerBehavior= DcMotor.ZeroPowerBehavior.BRAKE
    }
    /**
     * TODO: Write a function to handle the physics and power the motor
     */
    fun Drive(){}
    /**
     * TODO: Write a function that you can repeat forever in the opMode
     */
    fun DriveCommand(){}

 }