package org.firstinspires.ftc.teamcode.IronNestUNCODE;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

@Autonomous  (name = "The auto blue")
public class UncodeAutoBlue extends LinearOpMode{

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
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
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        DcMotorEx FL = hardwareMap.get(DcMotorEx.class, "FrontL");
        DcMotorEx FR = hardwareMap.get(DcMotorEx.class, "FrontR");
        DcMotorEx BR = hardwareMap.get(DcMotorEx.class, "BackR");
        DcMotorEx BL = hardwareMap.get(DcMotorEx.class, "BackL");
        DcMotorEx OutL = hardwareMap.get(DcMotorEx.class, "outtakeL");
        DcMotorEx OutR = hardwareMap.get(DcMotorEx.class, "outtakeR");
        DcMotorEx In = hardwareMap.get(DcMotorEx.class, "intake");
        Servo liftL = hardwareMap.get(Servo.class, "LiftL");
        Servo liftR = hardwareMap.get(Servo.class, "LiftR");
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);

        initAprilTag();


        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
                FL.setPower(0.375); FR.setPower(0.375); BL.setPower(0.375); BR.setPower(0.375);
                sleep(1850);
                Reached_target_position = true;
                FL.setPower(0); FR.setPower(0); BL.setPower(0); BR.setPower(0);
            }else {
                FL.setPower(0); FR.setPower(0); BL.setPower(0); BR.setPower(0);
                sleep(1000);
                if (timer.seconds()<3 && iteration<3){
                    OutL.setPower(-0.95); OutR.setPower(0.95);
                    In.setPower(-1);
                    sleep(15);
                    liftL.setPosition(0.01);
                    liftR.setPosition(0.99);
                    telemetry.addData("Status", "Outtake");
                }else {
                    liftL.setPosition(0.22);
                    liftR.setPosition(0.78);
                    sleep(1000);
                    timer.reset();
                    iteration += 1;
                    telemetry.addData("Status", "Outtake Complete");
                    if(iteration == 3){
                        FL.setPower(0.75); FR.setPower(-0.75); BL.setPower(-0.75); BR.setPower(0.75);
                        sleep(750);
                        break;}
                }
                telemetry.update();
            }
        }
     }
}

