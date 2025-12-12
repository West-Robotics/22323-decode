package org.firstinspires.ftc.teamcode.IronNestUNCODE;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.Drivetrain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.util.control.Controller;
import org.firstinspires.ftc.teamcode.util.control.PIDController;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.util.Range;
import com.bylazar.configurables.annotations.Configurable;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Configurable
public abstract class  Base_Robot extends LinearOpMode {
    public DcMotorEx FR, FL, BR, BL, OutL, OutR, In;
    public AprilTagProcessor apriltag;

    public VisionPortal visionPortal;
    final boolean USE_WEBCAM = true;
    public static final int DESIRED_TAG_ID = -1;
    public AprilTagDetection desiredTag;

    public Servo liftL, liftR;

    public Controller Gamepad1, Gamepad2;

    public static double flywheelk_P = 0.001 ;
    public static double flywheelk_D = 0.0000000000001;
    public  static double flywheelk_i = 0.0001;

    private PIDController leftFlywheelController;
    private PIDController rightFlywheelController;
    private double targetFlywheelVelocity = 0;
    public double leftFlywheelPower = 0;
    public double rightFlywheelPower = 0;
    public boolean targetFound= false;

    public void init_drivetrain(){
        this.FL = hardwareMap.get(DcMotorEx.class, "FrontL");
        this.FR = hardwareMap.get(DcMotorEx.class, "FrontR");
        this.BR = hardwareMap.get(DcMotorEx.class, "BackR");
        this.BL = hardwareMap.get(DcMotorEx.class, "BackL");
        this.liftL = hardwareMap.get(Servo.class, "LiftL");
        this.liftR = hardwareMap.get(Servo.class, "LiftR");
        this.In = hardwareMap.get(DcMotorEx.class, "intake");
        this.OutL = hardwareMap.get(DcMotorEx.class, "outtakeL");
        this.OutR = hardwareMap.get(DcMotorEx.class, "outtakeR");

        this.Gamepad1 = new Controller(gamepad1);
        this.Gamepad2 = new Controller(gamepad2);

        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        OutL.setDirection(DcMotorSimple.Direction.REVERSE);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
    public void moveFlywheels() {
        // Get the current velocity from the motors
        double currentLeftVelocity = OutL.getVelocity();
        double currentRightVelocity = OutR.getVelocity();
        // Calculate the power adjustment using the PID controllers
        leftFlywheelPower = leftFlywheelController.performPID(currentLeftVelocity);
        rightFlywheelPower = rightFlywheelController.performPID(currentRightVelocity);
        // Apply the calculated power to the motors
        OutL.setPower(leftFlywheelPower);
        OutR.setPower(rightFlywheelPower);
    }

    public void init_vision(){
            // Create the AprilTag processor by using a builder.
            apriltag = new AprilTagProcessor.Builder().build();

            // Adjust Image Decimation to trade-off detection-range for detection-rate.
            // e.g. Some typical detection data using a Logitech C920 WebCam
            // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
            // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
            // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
            // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
            // Note: Decimation can be changed on-the-fly to adapt during a match.
            apriltag.setDecimation(2);

            // Create the vision portal by using a builder.
            if (USE_WEBCAM) {
                visionPortal = new VisionPortal.Builder()
                        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                        .addProcessor(apriltag)
                        .build();
            } else {
                visionPortal = new VisionPortal.Builder()
                        .setCamera(BuiltinCameraDirection.BACK)
                        .addProcessor(apriltag)
                        .build();
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
    public void moveRobot(){
        double x = Gamepad1.left_stick_x;
        double y = -Gamepad1.left_stick_y;
        double yaw = -Gamepad1.right_stick_x;
        calculatePower(x,y,yaw);
    }

    private void calculatePower( double x, double y, double yaw){

        DcMotor FL = this.FL;
        DcMotor FR = this.FR;
        DcMotor BL = this.BL;
        DcMotor BR = this.BR;

        // Calculate wheel powers.
        double frontLeftPower = x - y - yaw;
        double frontRightPower = x + y + yaw;
        double backLeftPower = x + y - yaw;
        double backRightPower = x - y + yaw;
        // this is the configuration for the robot wheels during auto. If the robot already doesn't behave as you want it to then you should change this.

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Send powers to the wheels.
        FL.setPower(frontLeftPower);
        FR.setPower(frontRightPower);
        BL.setPower(backLeftPower);
        BR.setPower(backRightPower);
    }
    public boolean lookForAprilTags(){
        desiredTag = null;
        List<AprilTagDetection> currentDetections = this.apriltag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    this.desiredTag = detection;
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
        if (targetFound) {
            return true;
        } else {
            return false;
        }
    }

    public double getTargetFlywheelVelocity() {
        return targetFlywheelVelocity;
    }

    public void setTargetFlywheelVelocity(double targetFlywheelVelocity) {
        this.targetFlywheelVelocity = targetFlywheelVelocity;
    }
}
