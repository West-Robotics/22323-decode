package org.firstinspires.ftc.teamcode.terrence;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.hardware.Gyro;
import org.firstinspires.ftc.teamcode.hardware.drive.DifferentialDrive;

//@Disabled
@TeleOp(name="Terrence: Teleop", group="Terrence")

public class TerrenceSource extends OpMode {

    private DifferentialDrive drive;
    private Gyro gyro;
    private Servo servo;
    private static final double HIGH_POS = 0.95;
    private static final double LOW_POS = 0.5;
    private static final double SLOW_MULTIPLIER = 0.2;

    private Controller controller;

    @Override
    public void init() {

        drive = new DifferentialDrive(this, hardwareMap);
        drive.debug();
        gyro = new Gyro(hardwareMap);
        servo = hardwareMap.get(Servo.class, "servo");
        controller = new Controller(gamepad1);

    }

    @Override
    public void loop() {

        controller.update();
        telemetry.addData("Gyro", gyro.getAngleRadians());
        drive.toggleArcade(controller.backOnce());
        if (controller.leftBumper()) {
            drive.drive(controller.left_stick_y * SLOW_MULTIPLIER, controller.right_stick_y * SLOW_MULTIPLIER, controller.right_stick_x * SLOW_MULTIPLIER);
        } else {
            drive.drive(controller.left_stick_y, controller.right_stick_y, controller.right_stick_x);
        }
        if (controller.X()) servo.setPosition(HIGH_POS);
        else servo.setPosition(LOW_POS);

        telemetry.update();

    }

}
