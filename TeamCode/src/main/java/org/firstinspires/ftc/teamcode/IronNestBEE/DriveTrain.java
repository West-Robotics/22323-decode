package org.firstinspires.ftc.teamcode.IronNestBEE;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * All things related to the driving and the drivetrain are made here.
 * This class acts as a Subsystem in the Command-Based structure.
 */
public class DriveTrain {
    private final DcMotorEx FR, FL, BR, BL;

    /**
     * Initialize all motors from the hardware map.
     * @param hardwareMap The OpMode's hardware map.
     */
    public DriveTrain(HardwareMap hardwareMap) {
        this.FL = hardwareMap.get(DcMotorEx.class, "FrontL");
        this.FR = hardwareMap.get(DcMotorEx.class, "FrontR");
        this.BR = hardwareMap.get(DcMotorEx.class, "BackR");
        this.BL = hardwareMap.get(DcMotorEx.class, "BackL");

        this.FR.setDirection(DcMotorSimple.Direction.FORWARD);
        this.BR.setDirection(DcMotorSimple.Direction.FORWARD);
        this.FL.setDirection(DcMotorSimple.Direction.REVERSE);
        this.BL.setDirection(DcMotorSimple.Direction.REVERSE);

        this.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Sets powers to the motors based on holonomic drive inputs.
     * @param x Strafe input
     * @param y Forward input
     * @param yaw Turn input
     */
    public void drive(double x, double y, double yaw) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(yaw), 1);
        double frontLeftPower = (y + x + yaw) / denominator;
        double backLeftPower = (y - x + yaw) / denominator;
        double frontRightPower = (y - x - yaw) / denominator;
        double backRightPower = (y + x - yaw) / denominator;

        FL.setPower(frontLeftPower);
        FR.setPower(frontRightPower);
        BL.setPower(backLeftPower);
        BR.setPower(backRightPower);
    }

    /**
     * Stops all drive motors.
     */
    public void stop() {
        drive(0, 0, 0);
    }

    /**
     * Command factory for continuous TeleOp drive.
     * @param gamepad The gamepad to read stick inputs from.
     * @return A looping command that updates motor powers.
     */
    public Command Drive(Gamepad gamepad) {
        return Command.build()
                .setExecute(() -> {
                    double x = gamepad.left_stick_x;
                    double y = -gamepad.left_stick_y;
                    double yaw = gamepad.right_stick_x;
                    drive(x, y, yaw);
                })
                .requiring(this);
    }

    /**
     * Command factory to stop the drivetrain.
     * @return A command that stops all motors once.
     */
    public Command Stop() {
        return Command.build()
                .setStart(this::stop)
                .requiring(this);
    }

    public DcMotorEx getBL() {
        return BL;
    }

    public DcMotorEx getBR() {
        return BR;
    }

    public DcMotorEx getFL() {
        return FL;
    }

    public DcMotorEx getFR() {
        return FR;
    }
}
