package org.firstinspires.ftc.teamcode.IronNestUNCODE;

// Add this new import statement
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import  org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.bylazar.camerastream.PanelsCameraStream;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name= "TestStreamAndDetect", group = "TeleOp")
public class AprilTagStreamingMode extends LinearOpMode {
    private static final int DESIRED_TAG_ID = -1 ;
    public AprilTagStreamProcessor apriltagStreamProcessor;
    private void init_vision() {
        // Create the AprilTag processor.
        AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation.
        aprilTagProcessor.setDecimation(2);

        // Create your new combined processor
        apriltagStreamProcessor = new AprilTagStreamProcessor(aprilTagProcessor);
        // Create the vision portal using the new combined processor.
        boolean USE_WEBCAM = true;
        VisionPortal visionPortal;
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(apriltagStreamProcessor) // Use the new processor here
                    .enableLiveView(false)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(apriltagStreamProcessor) // And here
                    .enableLiveView(false)
                    .build();
        }

        // Start the camera stream to FTC Panels
        PanelsCameraStream.INSTANCE.startStream(apriltagStreamProcessor, null);
    }

    public void lookForAprilTags() {
        AprilTagDetection desiredTag = null;
        boolean targetFound = false;
        // Get detections from your new processor
        List<AprilTagDetection> currentDetections = this.apriltagStreamProcessor.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    // Yes, we want to use this tag.
                    telemetry.addLine("found tag" + detection.id);
                    telemetry.addLine("distance to the tag"+detection.ftcPose.range);
                    telemetry.update();
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    targetFound = false;
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                targetFound = false;
            }
        }
    }
    public void stopStreaming() {
        PanelsCameraStream.INSTANCE.stopStream();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        init_vision();
        while (opModeIsActive() && !isStopRequested()){
            lookForAprilTags();
            telemetry.update();
            if (isStopRequested()){
                stopStreaming();
            }
        }
    }
}

