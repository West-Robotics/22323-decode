package org.firstinspires.ftc.teamcode.util.control;

import com.qualcomm.robotcore.hardware.DcMotor;

public class DcMotorUtils {
    // Move the motor to the position specified, in ticks
    // Except it only sets the power and you have to run it in the loop() thing, because yes
    // NOTE: in the future add lambdas so that you can pass your preferred decel func to it
    static public void moveByTicks(DcMotor motor, double desiredTicks, double startDec, double tolerance) {
        double diff = getDifference(motor, desiredTicks);
        // Deceleration: min(x * 1/startDec, 1) where x = difference
        // Hacky workaround for startDec being zero
        if (startDec == 0.0) {
            startDec = 0.1;
        }
        if (Math.abs(diff) > tolerance ) {
            int direction = (int) (diff/Math.abs(diff));
            motor.setPower(Math.min(Math.abs(diff)*(1.0/startDec), 1) * direction);
        } else {
            motor.setPower(0);
        }
    }

    static public void moveByDistance(DcMotor motor, double distance, double ticksPerRev, double circumference, double startDec, int tolerance) {
        moveByTicks(motor, ((distance/circumference) * ticksPerRev), startDec, tolerance);
    }

    static public double getDifference(DcMotor motor, double desiredTicks) { return desiredTicks - getCurrentTicks(motor); }
    static public int getCurrentTicks(DcMotor motor) { return motor.getCurrentPosition(); }
    static public boolean arrived(DcMotor motor, double desiredTicks, double tolerance) { return (getDifference(motor, desiredTicks) < tolerance); }
}
