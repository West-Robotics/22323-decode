package org.firstinspires.ftc.teamcode.hardware.drive.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

public class OdometryGlobalCoordinatePosition implements Runnable {

    // Odometry wheels
    private final DcMotor vlEncoder, vrEncoder, hEncoder;

    // Thead run condition
    private boolean isRunning = true;

    // Position variables used for storage and calculations
    double vrEncoderPos = 0, vlEncoderPos = 0, hEncoderPos = 0,  orientationChange = 0;
    private double globalX = 0, globalY = 0, orientation = 0;
    private double prevVREncoderPos = 0, prevVLEncoderPos = 0, prevHEncoderPos = 0;

    // Algorithm constants
    private final double encoderWheelDist;
    private final double hEncoderTickPerDegreeOffset;

    // Encoder tick calculation
    private final double CPR = 360 * 4;
    private final double WHEEL_DIAMETER = 3;
    private final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    private final double COUNTS_PER_INCH = CPR / WHEEL_CIRCUMFERENCE;

    // Sleep time interval (milliseconds) for the position update thread
    private final int SLEEP_TIME = 75;

    // Files to access the algorithm constants
    private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    private int vlEncoderPosMultiplier = 1;
    private int vrEncoderPosMultiplier = 1;
    private int hEncoderPosMultiplier = 1;

    public OdometryGlobalCoordinatePosition(HardwareMap hwMap){

        // Assign the hardware map to the odometry wheels
        vlEncoder = hwMap.get(DcMotor.class, "frontRight");
        vrEncoder = hwMap.get(DcMotor.class, "backRight");
        hEncoder = hwMap.get(DcMotor.class, "backLeft");

        // Set direction of encoders
        vlEncoder.setDirection(DcMotor.Direction.FORWARD);
        hEncoder.setDirection(DcMotor.Direction.REVERSE);
        vrEncoder.setDirection(DcMotor.Direction.FORWARD);

        // Reverse encoder directions
        reverseLeftEncoder();
        reverseNormalEncoder();
        //reverseRightEncoder();

        encoderWheelDist = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim()) * COUNTS_PER_INCH;
        hEncoderTickPerDegreeOffset = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());

    }

    // Updates the global (x, y, theta) coordinate position of the robot using the odometry encoders
    private void globalCoordinatePositionUpdate(){

        // Get Current Positions
        vlEncoderPos = (vlEncoder.getCurrentPosition() * vlEncoderPosMultiplier);
        vrEncoderPos = (vrEncoder.getCurrentPosition() * vrEncoderPosMultiplier);

        double leftChange = vlEncoderPos - prevVLEncoderPos;
        double rightChange = vrEncoderPos - prevVREncoderPos;

        // Calculate Angle
        orientationChange = (leftChange - rightChange) / encoderWheelDist;
        orientation = orientation + orientationChange;

        // Get the components of the motion
        hEncoderPos = (hEncoder.getCurrentPosition() * hEncoderPosMultiplier);
        double rawHorizontalChange = hEncoderPos - prevHEncoderPos;
        double horizontalChange = rawHorizontalChange - (orientationChange * hEncoderTickPerDegreeOffset);

        // Average distance
        double p = ((rightChange + leftChange) / 2);

        // Calculate and update the position values
        globalX = globalX + (p * Math.sin(orientation) + horizontalChange * Math.cos(orientation));
        globalY = globalY + (p * Math.cos(orientation) - horizontalChange * Math.sin(orientation));

        // Set previous encoder positions
        prevVLEncoderPos = vlEncoderPos;
        prevVREncoderPos = vrEncoderPos;
        prevHEncoderPos = hEncoderPos;

    }

    // Returns the robot's global x coordinate
    public double getX(){ return globalX / COUNTS_PER_INCH; }

    // Returns the robot's global y coordinate
    public double getY(){ return globalY / COUNTS_PER_INCH; }

    // Get encoder positions
    public int getVl() { return vlEncoder.getCurrentPosition(); }
    public int getVr() { return vrEncoder.getCurrentPosition(); }
    public int getH() { return hEncoder.getCurrentPosition(); }

    // Get wheel distance
    public double getWheelDist() { return encoderWheelDist; }

    // Returns the robot's global orientation
    public double getOrientation() { return Math.toDegrees(orientation) % 360; }

    // Reset global position
    public void resetPosition(boolean button) {

        if (button) {

            globalX = 0;
            globalY = 0;
            orientation = 0;

            vlEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            vrEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }

    }

    // Reset global position
    public void resetPosition() {

        globalX = 0;
        globalY = 0;
        orientation = 0;

        vlEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vrEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    // Stops the position update thread
    public void stop(){ isRunning = false; }

    private void reverseLeftEncoder(){

        if (vlEncoderPosMultiplier == 1) {

            vlEncoderPosMultiplier = -1;

        } else {

            vlEncoderPosMultiplier = 1;

        }

    }

    private void reverseRightEncoder(){

        if (vrEncoderPosMultiplier == 1) {

            vrEncoderPosMultiplier = -1;

        } else {

            vrEncoderPosMultiplier = 1;

        }

    }

    private void reverseNormalEncoder(){

        if (hEncoderPosMultiplier == 1) {

            hEncoderPosMultiplier = -1;
        } else {

            hEncoderPosMultiplier = 1;

        }

    }

    @Override
    public void run() {

        while (isRunning) {

            globalCoordinatePositionUpdate();
            try {

                Thread.sleep(SLEEP_TIME);

            } catch (InterruptedException e) {

                e.printStackTrace();

            }

        }

    }

}
