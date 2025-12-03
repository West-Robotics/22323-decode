package org.firstinspires.ftc.teamcode.IronNestUNCODE;

import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorTargetVelocityCommand;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.control.Controller;
import org.firstinspires.ftc.teamcode.util.control.PIDController;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;
import java.util.Locale;

@TeleOp(name="V2-tele")
public class UncodeTeleV2 extends LinearOpMode{
    private AprilTagProcessor aprilTag; // AprilTag processor object
    private VisionPortal visionPortal; // Vision portal object

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    private static final int TargetVelocity = 28300;
    private static final int Speed_Gain = 2;
    private static final double Braking_gain = 1.2;

    private  PIDController VelocityControl;
    @Override
        public void runOpMode() {

            // Initialize the hardware variables. Note that the strings used here must correspond
            // to the names assigned during the robot configuration step on the DS or RC devices.
            DcMotor FL = hardwareMap.get(DcMotor.class, "FrontL");
            DcMotor FR = hardwareMap.get(DcMotor.class, "FrontR");
            DcMotor BR = hardwareMap.get(DcMotor.class, "BackR");
            DcMotor BL = hardwareMap.get(DcMotor.class, "BackL");
            DcMotor In = hardwareMap.get(DcMotor.class, "intake");
            DcMotorEx OutL = hardwareMap.get(DcMotorEx.class, "outtakeL");
            DcMotorEx OutR = hardwareMap.get(DcMotorEx.class, "outtakeR");
            Servo liftL = hardwareMap.get(Servo.class, "LiftL");
            Servo liftR = hardwareMap.get(Servo.class, "LiftR");

            FR.setDirection(DcMotorSimple.Direction.FORWARD);
            BR.setDirection(DcMotorSimple.Direction.REVERSE);
            FL.setDirection(DcMotorSimple.Direction.FORWARD);
            BL.setDirection(DcMotorSimple.Direction.FORWARD);

            FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            OutR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            OutL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            OutR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            OutL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            initAprilTag();
            VelocityControl.enable();
            VelocityControl.setSetpoint(TargetVelocity);
            VelocityControl.setOutputRange(0,1);


            Controller Gamepad1 = new Controller(gamepad1);
            VelocityControl= new PIDController(Speed_Gain,0, Braking_gain);
            waitForStart();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                double Actual_velocityL = OutL.getVelocity();
                double Actual_velocityR = OutR.getVelocity();

                telemetryAprilTag();
                double y = -Gamepad1.left_stick_y;
                double x = Gamepad1.left_stick_x;
                double rx = Gamepad1.right_stick_x;

                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;

                // Slow mode
                if(Gamepad1.rightBumper()){
                    FL.setPower(frontLeftPower/6);
                    BL.setPower(backLeftPower/6);
                    FR.setPower(frontRightPower/6);
                    BR.setPower(backRightPower/6);
                }else{FL.setPower(frontLeftPower);
                    BL.setPower(backLeftPower);
                    FR.setPower(frontRightPower);
                    BR.setPower(backRightPower);
                }

                // Servo position
                if(Gamepad1.A()||gamepad2.a){
                    liftL.setPosition(0.01);
                    liftR.setPosition(0.99);
                }else{
                        liftL.setPosition(0.18);
                        liftR.setPosition(0.7);
                }

                // Intake controls
                if(Gamepad1.left_trigger>.2||gamepad2.left_trigger>.2){
                    In.setPower(-1);
                }else if(Gamepad1.leftBumper()||gamepad2.left_bumper){
                    In.setPower(0.25);
                }else{
                    In.setPower(0);
                }


                // Outtake controls
                if(Gamepad1.right_trigger>0.5||gamepad2.right_trigger>0.5){
                    OutL.setPower(1);
                    OutR.setPower(-1);
                } else{
                    OutL.setPower(0);
                    OutR.setPower(0);
                }
                Gamepad1.update();
                telemetry.update();
            }


    }
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

}
