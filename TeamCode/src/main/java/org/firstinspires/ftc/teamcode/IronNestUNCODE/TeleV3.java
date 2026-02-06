package org.firstinspires.ftc.teamcode.IronNestUNCODE;

import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Configurable
@TeleOp(name = "TeleV3", group = "TeleOp")
public class TeleV3 extends Base_Robot{
    @Override
    public void runOpMode() throws InterruptedException {
        // initialize everything ...
        init_motor();
        init_flywheels();
        setFlywheelVelocity(2660);
        init_vision();
        if(USE_WEBCAM)
            setManualExposure();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()){
            targetFound = false;
            desiredTag = null;
            // basic functionality
            controlFlywheels();
            manageIntake();
            manage_servos();

            // look for apriltags without breaking the code. ( makes sure that the apriltag is actually valid before doing anything)

                if (gamepad1.right_bumper) {
                    approachApriltags();
                }else if (gamepad1.right_bumper && desiredTag == null) {
                    moveRobot();
                    panelsTelemetry.addLine("You tried but there was no aprilTag");
                } else {
                    moveRobot();
                }

            updateGamepads();
            // Crucial detecting information about the apriltag..
            panelsTelemetry.addData("desired distance away from target ", DESIRED_DISTANCE);
            panelsTelemetry.addData("target found var: ", targetFound);
            panelsTelemetry.addData("Our setpoint; ", leftFlywheelController.getSetpoint());

            // target found must be true desired tag should be something before we can use the information.
            if (targetFound && desiredTag!= null) {
                    panelsTelemetry.addData("Our actual  distance to the apriltag: ", desiredTag.ftcPose.range);
                } else {
                    panelsTelemetry.addData("Our distance to the apriltag", "No apriltag detected");
            }
            // tuning aprilTag information.
            panelsTelemetry.addData("Auto aim forward power supply ", drive);
            panelsTelemetry.update(telemetry);
            telemetry.update();
            if(isStopRequested()){
                stopStreaming();
            }
        }
    }
}
