package org.firstinspires.ftc.teamcode.IronNestUNCODE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Locale;

@Autonomous  (name = "Scrim auto")
public class UncodeAuto extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    }



    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format(Locale.US, "\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format(Locale.US, "XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format(Locale.US, "PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format(Locale.US, "RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format(Locale.US, "\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format(Locale.US, "Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()
    @Override
    public void runOpMode() {
        ElapsedTime timer ;
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        DcMotor FL = hardwareMap.get(DcMotor.class, "FrontL");
        DcMotor FR = hardwareMap.get(DcMotor.class, "FrontR");
        DcMotor BR = hardwareMap.get(DcMotor.class, "BackR");
        DcMotor BL = hardwareMap.get(DcMotor.class, "BackL");
        DcMotor OutL = hardwareMap.get(DcMotor.class, "outtakeL");
        DcMotor OutR = hardwareMap.get(DcMotor.class, "outtakeR");
        DcMotor In = hardwareMap.get(DcMotor.class, "intake");
        Servo liftL = hardwareMap.get(Servo.class, "LiftL");
        Servo liftR = hardwareMap.get(Servo.class, "LiftR");
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        
        initAprilTag();
        
        int iteration  = 0 ;
        boolean Reached_target_position = false;
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        timer = new ElapsedTime();
        waitForStart();
        timer.reset();
        while (opModeIsActive()){
            telemetryAprilTag();

            if (timer.seconds()<0.5 && !Reached_target_position){
                FL.setPower(0.75); FR.setPower(0.75); BL.setPower(0.75); BR.setPower(0.75);
                sleep(700);
                Reached_target_position = true;
                FL.setPower(-0.75); FR.setPower(-0.75); BL.setPower(-0.75); BR.setPower(-0.75);
                sleep(10);
                FL.setPower(0); FR.setPower(0); BL.setPower(0); BR.setPower(0);
            }else {
                FL.setPower(0); FR.setPower(0); BL.setPower(0); BR.setPower(0);
                sleep(1000);
                    if (timer.seconds()<3 && iteration<3){
                        OutL.setPower(-1); OutR.setPower(1);
                        In.setPower(-1);
                        sleep(15);
                        liftL.setPosition(0.01);
                        liftR.setPosition(0.99);
                        telemetry.addData("Status", "Outtake");
                    }else {
                        liftL.setPosition(0.15);
                        liftR.setPosition(0.85);
                        sleep(1000);
                        timer.reset();
                        iteration += 1;
                        telemetry.addData("Status", "Outtake Comple");
                        if(iteration == 3){
                            FL.setPower(-0.75); FR.setPower(0.75); BL.setPower(0.75); BR.setPower(-0.75);
                            sleep(500);
                            break;}
            }
                telemetry.update();
        }
    }
}
}