package org.firstinspires.ftc.teamcode.IronNestUNCODE;

import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.gamepad.GamepadManager;
import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.util.control.PIDController;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.Range;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Configurable
public abstract class  Base_Robot extends LinearOpMode {
    public static double MAX_AUTO_TURN = 1, MAX_AUTO_STRAFE = 0.5, MAX_AUTO_SPEED = 1;
    public static double DESIRED_DISTANCE = 48,SPEED_GAIN = 0.03,TURN_GAIN = 0.03;
    public static double flywheelk_P = 0.001,flywheelk_D = 0.0000000000001, flywheelk_i = 0.0001;
    private static double STRAFE_GAIN = 0.015;
    public DcMotorEx FR, FL, BR, BL, OutL, OutR, In;
    public VisionPortal visionPortal;
    final boolean USE_WEBCAM = true;
    public static final int DESIRED_TAG_ID = -1;
    public AprilTagDetection desiredTag;
    public Servo liftL, liftR;
    private PIDController leftFlywheelController;
    private PIDController rightFlywheelController;
    private double targetFlywheelVelocity = 0;
    public double leftFlywheelPower = 0;
    public double rightFlywheelPower = 0;
    public boolean targetFound= false;
    public TelemetryManager panelsTelemetry;
    public GamepadManager g1_panels_manager;
    public GamepadManager g2_panels_manager;
    public double rangeError, headingError, yawError;
    public double drive,turn,strafe;
    public AprilTagStreamProcessor apriltagStreamProcessor;
    /*
    TO DO:
    - calibrate the camera using the tutorial found here: https://ftc-docs.firstinspires.org/en/latest/programming_resources/vision/camera_calibration/camera-calibration.html
    - test and run teleop. make sure nothing is inverted and basic functionality works
    - test auto aim:
      - graph the variables in ftc panels and get a sense of what is happening and why the auto aim keeps going straight into the apriltag even after reaching desired distance

     */

    public void init_motor(){
        this.FL = hardwareMap.get(DcMotorEx.class, "FrontL");
        this.FR = hardwareMap.get(DcMotorEx.class, "FrontR");
        this.BR = hardwareMap.get(DcMotorEx.class, "BackR");
        this.BL = hardwareMap.get(DcMotorEx.class, "BackL");
        this.liftL = hardwareMap.get(Servo.class, "LiftL");
        this.liftR = hardwareMap.get(Servo.class, "LiftR");
        this.In = hardwareMap.get(DcMotorEx.class, "intake");
        this.OutL = hardwareMap.get(DcMotorEx.class, "outtakeL");
        this.OutR = hardwareMap.get(DcMotorEx.class, "outtakeR");

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        OutL.setDirection(DcMotorSimple.Direction.REVERSE);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // 2. Initialize the PanelsGamepad managers
        g1_panels_manager = PanelsGamepad.INSTANCE.getFirstManager();
        g2_panels_manager = PanelsGamepad.INSTANCE.getSecondManager();
    }
    public void init_flywheels() {
        // Initialize the PID controllers with your chosen gains
        // We are not using an Integral term (Ki=0.0) for this simple velocity controller
        leftFlywheelController = new PIDController(flywheelk_P,flywheelk_i, flywheelk_D);
        rightFlywheelController = new PIDController(flywheelk_P, flywheelk_i, flywheelk_D);

        // Set output range for the PID controllers to match motor power range
        leftFlywheelController.setOutputRange(0, 1.0);
        rightFlywheelController.setOutputRange(0, 1.0);

        // Enable the PID controllers
        leftFlywheelController.enable();
        rightFlywheelController.enable();
    }

    /**
     * Sets the desired target velocity for the flywheels.
     * @param velocity The target velocity in ticks per second.
     */
    public void setFlywheelVelocity(double velocity) {
        targetFlywheelVelocity = velocity;
        leftFlywheelController.setSetpoint(targetFlywheelVelocity);
        rightFlywheelController.setSetpoint(targetFlywheelVelocity);
    }

    /**
     * This method MUST be called in the main loop of your OpMode.
     * It calculates and applies the necessary power to the flywheels to maintain the target velocity.
     */
    public void controlFlywheels() {
        // Get the current velocity from the motors
        double currentLeftVelocity = OutL.getVelocity();
        double currentRightVelocity = OutR.getVelocity();
        // Calculate the power adjustment using the PID controllers
        if (gamepad1.right_trigger>0) {
            leftFlywheelPower = leftFlywheelController.performPID(currentLeftVelocity);
            rightFlywheelPower = rightFlywheelController.performPID(currentRightVelocity);
            // Apply the calculated power to the motors
            OutL.setPower(leftFlywheelPower);
            OutR.setPower(rightFlywheelPower);
        }
        else {
            OutL.setPower(0);
            OutR.setPower(0);
        }

    }
    public void manageIntake(){
        if(gamepad1.left_trigger>.2||gamepad2.left_trigger>.2){
            In.setPower(-1);
        }else if(gamepad1.left_bumper||gamepad2.left_bumper){
            In.setPower(0.25);
        }else{
            In.setPower(0);
        }
    }
    public void manage_servos(){
        if(gamepad1.a){
            liftL.setPosition(0.01);
            liftR.setPosition(0.99);
        }else {
            liftR.setPosition(0.785);
            liftL.setPosition(0.215);
        }
    }
    public void init_vision() {
        // Create the AprilTag processor.
        AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder()
                .build();

        // Adjust Image Decimation.
        aprilTagProcessor.setDecimation(3);

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

    public void  setManualExposure() {
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
    public void moveRobot(){
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double yaw = gamepad1.right_stick_x;
        calculatePower(x,y,yaw);
    }

    private void calculatePower(double x, double y, double yaw){

        DcMotor FL = this.FL;
        DcMotor FR = this.FR;
        DcMotor BL = this.BL;
        DcMotor BR = this.BR;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(yaw), 1);
        double frontLeftPower = (y + x + yaw) / denominator;
        double backLeftPower = (y - x + yaw) / denominator;
        double frontRightPower = (y - x - yaw) / denominator;
        double backRightPower = (y + x - yaw) / denominator;

        // Send powers to the wheels.
        FL.setPower(frontLeftPower);
        FR.setPower(frontRightPower);
        BL.setPower(backLeftPower);
        BR.setPower(backRightPower);
    }
    public void approachApriltags(){
        lookForAprilTags();
        if (desiredTag != null && gamepad1.right_bumper){

            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            rangeError      = (this.desiredTag.ftcPose.range - DESIRED_DISTANCE);
            headingError    = this.desiredTag.ftcPose.bearing;
            yawError        = this.desiredTag.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            calculatePower(0,-drive,-turn);
        }
        else{
            calculatePower(0,0,0);
        }
    }
    public void lookForAprilTags() {
        targetFound = false;
        desiredTag = null;
        List<AprilTagDetection> currentDetections = this.apriltagStreamProcessor.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if you CAN calculate the distance to the AprilTag.
            if (detection.ftcPose != null) {
                //  Check to see if we want to track towards this tag.
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    // Yes, we want to use this tag.
                    telemetry.addLine("found tag" + detection.id);
                    telemetry.addLine("distance to the tag"+detection.ftcPose.range);
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    targetFound = false;
                    desiredTag = null;
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                targetFound = false;
                desiredTag = null;
            }
        }
    }
    public double getTargetFlywheelVelocity() {
        return targetFlywheelVelocity;
    }

    public void setTargetFlywheelVelocity(double targetFlywheelVelocity) {
        this.targetFlywheelVelocity = targetFlywheelVelocity;
    }
    public void updateGamepads() {
        g1_panels_manager.asCombinedFTCGamepad(gamepad1);
        g2_panels_manager.asCombinedFTCGamepad(gamepad2);
    }
    public void stopStreaming() {
        PanelsCameraStream.INSTANCE.stopStream();
    }

}
