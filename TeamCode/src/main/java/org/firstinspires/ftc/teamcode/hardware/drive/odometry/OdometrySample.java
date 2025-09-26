package org.firstinspires.ftc.teamcode.hardware.drive.odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Global Coordinate Position Test", group = "Calibration")
public class OdometrySample extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Init complete
        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        // Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        OdometryGlobalCoordinatePosition globalPositionUpdate = new OdometryGlobalCoordinatePosition(hardwareMap);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.resetPosition();

        while (opModeIsActive()) {

            // Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.getX());
            telemetry.addData("Y Position", globalPositionUpdate.getY());
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.getOrientation());
            telemetry.addData("Vl", globalPositionUpdate.getVl());
            telemetry.addData("Vr", globalPositionUpdate.getVr());
            telemetry.addData("H", globalPositionUpdate.getH());
            telemetry.addData("Wheel Dist", globalPositionUpdate.getWheelDist());
            telemetry.update();

        }

        // Stop the thread
        globalPositionUpdate.stop();

    }

}