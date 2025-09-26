package org.firstinspires.ftc.teamcode.hardware.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.BaseHardware;

public class DifferentialDrive extends BaseHardware {

    private DcMotor leftDrive;
    private DcMotor rightDrive;

    private boolean isArcade = true;
    private boolean isSlow = false;
    private static final double SLOW_MULTIPLIER = 1.5;

    public DifferentialDrive(OpMode opMode, HardwareMap hwMap) {

        super(opMode);
        setup(hwMap);

    }

    public DifferentialDrive(LinearOpMode opMode, HardwareMap hwMap) {

        super(opMode);
        setup(hwMap);

    }

    private void setup(HardwareMap hwMap) {

        leftDrive = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setPower(0);
        rightDrive.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    // Toggle arcade and tank drive
    public void toggleArcade(boolean button) {

        if (button) isArcade = !isArcade;

    }

    // Toggle slow mode
    public void toggleSlow(boolean button) {

        if (button) isSlow = !isSlow;

    }

    // Arcade and tank drive
    public void drive(double leftStickY, double rightStickY, double rightStickX) {

        double left;
        double right;

        if (isArcade) {

            // Arcade drive
            left = -leftStickY + rightStickX;
            right = -leftStickY - rightStickX;

            double max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                left /= max;
                right /= max;
            }

        } else {

            // Tank drive
            left = -leftStickY;
            right = -rightStickY;

        }

        leftDrive.setPower(isSlow ? left / SLOW_MULTIPLIER : left);
        rightDrive.setPower(isSlow ? right / SLOW_MULTIPLIER : right);

        print("Left: ", getEncoderValues()[0]);
        print("Right: ", getEncoderValues()[1]);

    }

    public int[] getEncoderValues() {

        return new int[]{leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition()};

    }

}