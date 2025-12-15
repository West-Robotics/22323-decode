package org.firstinspires.ftc.teamcode.IronNestUNCODE;

import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.vision.VisionPortal;

@Configurable
@TeleOp(name = "TeleV3", group = "TeleOp")
public class TeleV3 extends Base_Robot{
    private final TestCameraStream.Processor processor = new TestCameraStream.Processor();
    @Override
    public void runOpMode() throws InterruptedException {
        new VisionPortal.Builder().addProcessor(this.processor).setCamera(BuiltinCameraDirection.BACK).build();
        PanelsCameraStream.INSTANCE.startStream(this.processor, null);
        init_motor();
        init_flywheels();
        setFlywheelVelocity(2660);
        init_vision();
        if(USE_WEBCAM)
            setManualExposure();
        waitForStart();
        while (opModeIsActive()){
            controlFlywheels();
            manageIntake();
            manage_servos();
            try {
                if (gamepad1.right_bumper)
                    approachApriltags();
                else
                    moveRobot();
            } catch (NullPointerException e){
                panelsTelemetry.addLine("No apriltag detected");
                targetFound = false;
                panelsTelemetry.update(telemetry);
                moveRobot();
            }
            updateGamepads();
            panelsTelemetry.addData("desired distance away from target ", DESIRED_DISTANCE);
            try {
                panelsTelemetry.addData("Our distance to the apriltag ", desiredTag.ftcPose.range);
            } catch (NullPointerException e){
                panelsTelemetry.addData("Our distance to the apriltag", "No apriltag detected");
            }
            panelsTelemetry.addData("Auto aim forward power supply ", drive);
            panelsTelemetry.update(telemetry);
            telemetry.update();
        }
        if (isStopRequested()){
            PanelsCameraStream.INSTANCE.stopStream();
        }
    }
}
