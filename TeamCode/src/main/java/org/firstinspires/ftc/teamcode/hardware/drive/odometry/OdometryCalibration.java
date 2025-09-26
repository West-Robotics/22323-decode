package org.firstinspires.ftc.teamcode.hardware.drive.odometry;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.hardware.Gyro;

import java.io.File;

@TeleOp(name = "Odometry System Calibration", group = "Calibration")
public class OdometryCalibration extends LinearOpMode {

    // Drive motors
    private DcMotor frontRight, backRight, frontLeft, backLeft;

    // Odometry Wheels
    private DcMotor verticalLeft, verticalRight, horizontal;

    // Gyro
    private Gyro gyro;

    private final double PIVOT_SPEED = 0.5;

    // Encoder tick calculation
    private final double CPR = 360 * 4;
    private final double WHEEL_DIAMETER = 3;
    private final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    private final double COUNTS_PER_INCH = CPR / WHEEL_CIRCUMFERENCE;

    private ElapsedTime timer = new ElapsedTime();

    private double horizontalTickOffset = 0;

    // Text files to write the values to. The files are stored in the robot controller under Internal Storage\FIRST\settings
    private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize hardware map values
        initHardwareMap();

        // Initialize gyro
        gyro = new Gyro(hardwareMap);

        // Odometry System Calibration Init Complete
        telemetry.addData("Odometry System Calibration Status", "Init Complete");
        telemetry.update();

        waitForStart();

        // Begin calibration (if robot is unable to pivot at these speeds, please adjust the constant at the top of the code
        while (-gyro.getAngleDegrees() < 90 && opModeIsActive()){

            frontRight.setPower(-PIVOT_SPEED);
            backRight.setPower(-PIVOT_SPEED);
            frontLeft.setPower(PIVOT_SPEED);
            backLeft.setPower(PIVOT_SPEED);

            if (-gyro.getAngleDegrees() < 60) {

                setPowerAll(-PIVOT_SPEED, -PIVOT_SPEED, PIVOT_SPEED, PIVOT_SPEED);

            } else {

                setPowerAll(-PIVOT_SPEED / 2, -PIVOT_SPEED / 2, PIVOT_SPEED / 2, PIVOT_SPEED / 2);

            }

            telemetry.addData("IMU Angle", -gyro.getAngleDegrees());
            telemetry.update();
        }

        // Stop the robot
        setPowerAll(0, 0, 0, 0);
        timer.reset();
        while (timer.milliseconds() < 1000 && opModeIsActive()) {

            telemetry.addData("IMU Angle", -gyro.getAngleDegrees());
            telemetry.update();

        }

        // Record IMU and encoder values to calculate the constants for the global position algorithm
        double angle = -gyro.getAngleDegrees();

        // Encoder Difference is calculated by the formula (leftEncoder - rightEncoder)
        double encoderDifference = Math.abs(verticalLeft.getCurrentPosition()) + (Math.abs(verticalRight.getCurrentPosition()));

        double verticalEncoderTickOffsetPerDegree = encoderDifference / angle;

        double wheelBaseSeparation = (2 * 90 * verticalEncoderTickOffsetPerDegree) / (Math.PI * COUNTS_PER_INCH);

        horizontalTickOffset = horizontal.getCurrentPosition() / Math.toRadians(-gyro.getAngleDegrees());

        // Write the constants to text files
        ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(wheelBaseSeparation));
        ReadWriteFile.writeFile(horizontalTickOffsetFile, String.valueOf(horizontalTickOffset));

        while (opModeIsActive()) {

            telemetry.addData("Odometry System Calibration Status", "Calibration Complete");

            // Display calculated constants
            telemetry.addData("Wheel Base Separation", wheelBaseSeparation);
            telemetry.addData("Horizontal Encoder Offset", horizontalTickOffset);

            // Display raw values
            telemetry.addData("IMU Angle", -gyro.getAngleDegrees());
            telemetry.addData("Vertical Left Position", -verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical Right Position", verticalRight.getCurrentPosition());
            telemetry.addData("Horizontal Position", horizontal.getCurrentPosition());
            telemetry.addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree);

            // Update values
            telemetry.update();

        }

    }

    private void initHardwareMap(){

        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        verticalLeft = hardwareMap.get(DcMotor.class, "frontRight");
        verticalRight = hardwareMap.get(DcMotor.class, "backRight");
        horizontal = hardwareMap.get(DcMotor.class, "backLeft");

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);


        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();

    }

    private void setPowerAll(double rf, double rb, double lf, double lb){

        frontRight.setPower(rf);
        backRight.setPower(rb);
        frontLeft.setPower(lf);
        backLeft.setPower(lb);

    }

}
