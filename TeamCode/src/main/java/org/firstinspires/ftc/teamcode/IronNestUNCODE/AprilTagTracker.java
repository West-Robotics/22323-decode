package org.firstinspires.ftc.teamcode.IronNestUNCODE;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.control.Controller;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Configurable
public class AprilTagTracker{

    public static double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)

    public double getSPEED_GAIN() {
        return SPEED_GAIN;
    }

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    public static  double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)

    public double getSTRAFE_GAIN() {
        return STRAFE_GAIN;
    }

    public static double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)

    public double getTURN_GAIN() {
        return TURN_GAIN;
    }

    public static double TURN_GAIN = 0.01;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    public double getMAX_AUTO_SPEED() {
        return MAX_AUTO_SPEED;
    }

    public static double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)

   public double getMAX_AUTO_STRAFE() {
        return MAX_AUTO_STRAFE;
    }

   public static double MAX_AUTO_STRAFE = 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)

    public double getMAX_AUTO_TURN() {
        return MAX_AUTO_TURN;
    }

    public static double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    public  AprilTagProcessor apriltag;
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.

    private AprilTagDetection getDesiredTag() {
        return desiredTag;
    }

    private AprilTagDetection desiredTag;     // Used to hold the data for a detected AprilTag

    private final DcMotor frontLeftDrive;  //  Used to control the left front drive wheel
    private final DcMotor frontRightDrive;  //  Used to control the right front drive wheel
    private final DcMotor backLeftDrive;  //  Used to control the left back drive wheel
    private final DcMotor backRightDrive;  //  Used to control the right back drive wheel
    private Controller gamepad;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private TelemetryManager panelsTelemetry;

    public AprilTagTracker(HardwareMap hardwareMap, DcMotor frontLeftDrive, DcMotor frontRightDrive, DcMotor backLeftDrive, DcMotor backRightDrive, AprilTagProcessor aprilTag, Controller Gamepad1){
        this.frontLeftDrive = frontLeftDrive;
        this.frontRightDrive = frontRightDrive;
        this.backLeftDrive = backLeftDrive;
        this.backRightDrive = backRightDrive;
        this.hardwareMap = hardwareMap;

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        this.apriltag = aprilTag;
        this.gamepad = Gamepad1;
    }
    public void approachApriltagManually(){
        lookForApriltags();
        if (this.desiredTag != null && this.gamepad.leftBumper() && lookForApriltags()){
            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double  rangeError      = (this.desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double  headingError    = this.desiredTag.ftcPose.bearing;
            double  yawError        = this.desiredTag.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            double drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            moveRobot(-drive, -strafe, turn);
            panelsTelemetry.debug("String 1", "String 2", "String 3", new TestTelemetryPanelsStyle.CustomObject(11));
            panelsTelemetry.update();
        }
    }
    public boolean lookForApriltags() {
        boolean targetFound = false;
        this.desiredTag = null;

        // Step through the list of detected tags and look for a matching tag
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
    private void moveRobot ( double x, double y, double yaw){

        DcMotor FL = this.frontLeftDrive;
        DcMotor FR = this.frontRightDrive;
        DcMotor BL = this.backLeftDrive;
        DcMotor BR = this.backRightDrive;
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
    }
