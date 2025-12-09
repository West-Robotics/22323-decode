package org.firstinspires.ftc.teamcode.IronNestUNCODE;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.util.control.Controller;
import org.firstinspires.ftc.teamcode.util.control.PIDController;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

@Configurable
@TeleOp(name="V2-tele")
public class UncodeTeleV2 extends LinearOpMode{
    PIDController VelocityControlRight = new PIDController(Speed_Gain,0, Braking_gain);
    PIDController VelocityControlLeft = new PIDController(Speed_Gain,0, Braking_gain);
    private AprilTagProcessor aprilTag; // AprilTag processor object
    private VisionPortal visionPortal; // Vision portal object
    final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    public static final int TargetVelocityRight = 26300;
    public static final int TargetVelocityLeft = -26300;
    public static final double Speed_Gain = 1.2  ;
    private static final double Braking_gain = 1.2;

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

            VelocityControlRight.enable();
            VelocityControlRight.setSetpoint(TargetVelocityRight);
            VelocityControlRight.setOutputRange(0,1);
            VelocityControlLeft.enable();
            VelocityControlLeft.setSetpoint(TargetVelocityLeft);
            VelocityControlLeft.setOutputRange(0,1);



            initAprilTag();
            if (USE_WEBCAM)
                setManualExposure();  // Use low exposure time to reduce motion blur
            AprilTagTracker AutoMode = new AprilTagTracker(FL, FR, BL, BR, aprilTag); //sends these motors to the class and they will be controlled in the loop.
            Controller Gamepad1 = new Controller(gamepad1);
            waitForStart();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
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
                    double left_power = VelocityControlLeft.performPID(OutL.getVelocity());
                    double right_power = VelocityControlRight.performPID(OutR.getVelocity());
                    telemetry.addData("left power from PID ", left_power);
                    telemetry.addData("left wheel velocity", OutL.getVelocity());
                    telemetry.addData("right wheel power inputs", right_power);
                    telemetry.addData("right wheel velocity ", OutR.getVelocity());
                    if (left_power < 0) {
                        OutL.setPower(left_power);
                    }
                    if (right_power > 0) {
                        OutR.setPower(right_power);
                    }else{
                        telemetry.addLine("PID IS DOING WEIRD THINGS");
                    }
                    telemetry.update();
                } else{
                    OutL.setPower(0);
                    OutR.setPower(0);
                }

                AutoMode.approachApriltagManually();

                Gamepad1.update();
                telemetry.update();
            }


    }
    private void  setManualExposure() {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure(6, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(250);
            sleep(20);
        }

    }

    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }
}

