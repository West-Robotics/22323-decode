package org.firstinspires.ftc.teamcode.hardware.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.hardware.BaseHardware;
import org.firstinspires.ftc.teamcode.hardware.Gyro;
import org.firstinspires.ftc.teamcode.hardware.drive.odometry.OdometryGlobalCoordinatePosition;

public abstract class HolonomicDrive extends BaseHardware {

    public boolean thirdWheel = false;
    protected boolean reduceTurn = true;
    protected double inchesPerSecAtMax = 36.0;

    // Boolean function for moveUntil()
    public interface BoolCommand {

        boolean check();

    }

    // Motors
    protected DcMotor frontLeft;
    protected DcMotor frontRight;
    protected DcMotor backLeft;
    protected DcMotor backRight;

    // Gyro
    protected Gyro gyro;

    // Odometry
    protected OdometryGlobalCoordinatePosition odometry = null;

    // Modes
    protected boolean isDrivePOV = true;
    protected boolean isSlow = false;
    protected boolean isSquaredInputs = false;

    // For autonomous driving
    protected double wheelDiameter = 4;   // Diameter of driving wheels, default is 4
    protected double ticksPerRev = 1120;  // TPR of driving motors, defalut is 1120
    protected double currFlbr = 0; // Current theoretical x-value
    protected double currFrbl = 0; // Current theoretical y-value
    protected double currAngle = 0; // Current theoretical angle

    // For PID corrections
    protected PIDController pidTurn = new PIDController(0, 0, 0);
    protected PIDController pidFLBR = new PIDController(0, 0, 0);
    protected PIDController pidFRBL = new PIDController(0, 0, 0);
    protected PIDController pidAutoTurn = new PIDController(0, 0, 0);
    protected double prevAngle;
    protected double correction;
    protected boolean isPID = true;

    protected static final double SLOW_MULTIPLIER = 0.5;

    // Set pidDrive values
    protected void setPidTurn(double p, double i, double d) {

        pidTurn = new PIDController(p, i, d);

    }

    // Set pidSpeed values
    protected void setPidAutoSpeed(double p, double i, double d) {

        pidFLBR = new PIDController(p, i, d);
        pidFRBL = new PIDController(p, i, d);

    }

    // Set pidTurn values
    protected void setPidAutoTurn(double p, double i, double d) {

        pidAutoTurn = new PIDController(p, i, d);

    }

    // Call this function to stop correction
    protected void setPIDFalse() {

        isPID = false;

    }

    protected void setMotorDir(boolean flDir, boolean frDir, boolean blDir, boolean brDir) {

        if (flDir) frontLeft.setDirection(DcMotor.Direction.FORWARD);
        else frontLeft.setDirection(DcMotor.Direction.REVERSE);

        if (frDir) frontRight.setDirection(DcMotor.Direction.FORWARD);
        else frontRight.setDirection(DcMotor.Direction.REVERSE);

        if (blDir) backLeft.setDirection(DcMotor.Direction.FORWARD);
        else backLeft.setDirection(DcMotor.Direction.REVERSE);

        if (brDir) backRight.setDirection(DcMotor.Direction.FORWARD);
        else backRight.setDirection(DcMotor.Direction.REVERSE);

    }

    // Set the wheel diameter
    protected void setWheelDiameter(double wheelDiameter) {

        this.wheelDiameter = wheelDiameter;

    }

    // Set ticks per revolution
    protected void setTicksPerRev(double ticksPerRev) {

        this.ticksPerRev = ticksPerRev;

    }

    // Get odometry
    public void getOdometry(OdometryGlobalCoordinatePosition odometry) {

        this.odometry = odometry;

    }

    // For teleop
    public HolonomicDrive(OpMode opMode, HardwareMap hwMap) {

        super(opMode);

        gyro = new Gyro(hwMap, false);
        prevAngle = gyro.getAngleDegrees();
        gyro.reset();

        setupMotors(hwMap);

    }

    // For autonomous
    public HolonomicDrive(LinearOpMode opMode, HardwareMap hwMap) {

        super(opMode);

        gyro = new Gyro(hwMap);
        prevAngle = gyro.getAngleDegrees();
        gyro.reset();

        setupMotors(hwMap);

    }

    // Define and initialize motors
    private void setupMotors(HardwareMap hwMap) {

        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setPower(0);

        frontRight = hwMap.get(DcMotor.class, "frontRight");
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setPower(0);

        backLeft = hwMap.get(DcMotor.class, "backLeft");
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setPower(0);

        backRight = hwMap.get(DcMotor.class, "backRight");
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setPower(0);

        setupEncoders();

    }

    private void setupEncoders() {

        resetMotors();

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

/*        if (linearOpMode == null) {

            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        } else {
*/
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        }

    }

    public boolean isPOVMode() {
        return isDrivePOV;
    }

    // Toggle POV mode and field oriented mode
    public void togglePOV(boolean button) {

        if (button) {

            isDrivePOV = !isDrivePOV;
            gyro.reset();
            prevAngle = gyro.getAngleDegrees();

        }

    }

    // Enable squared inputs
    public void enableSquaredInputs() {

        isSquaredInputs = true;

    }

    // Toggle slow mode
    public void toggleSlow(boolean button) {

        if (button) isSlow = !isSlow;

    }

    public double celerate(double currentTime, double start, double max, double end, double accel, double decel) {
        // if after it's done
        if (end - currentTime < 0) {
            return 0;
        }
        // if before it's started
        if (currentTime - start < 0) {
            return 0;
        }
        return Math.min(Math.min(Math.sqrt(currentTime - start) * (1 / accel),
                                 max),
                        Math.sqrt(end - currentTime) * (1 / decel));
    }

    public double normie(double time, double end) {
        return celerate(time, 0, 1, end, 1, 1);
    }

    public void justX(double time, double endTime, int dir) {
        drive(celerate(time, 0.5, 0, endTime, 2, 2)*dir, 0, 0);
    }

    public void fakeRoadrunner(double time,
                               double xStart, double xMax, double xEnd, double xAccel, double xDecel, int xDir,
                               double yStart, double yMax, double yEnd, double yAccel, double yDecel, int yDir,
                               double turnStart, double turnMax, double turnEnd, double turnAccel, double turnDecel, int turnDir) {
        drive(celerate(time, xStart, xMax, xEnd, xAccel, xDecel)*xDir,
              celerate(time, yStart, yMax, yEnd, yAccel, yDecel)*yDir,
              celerate(time, turnStart, turnMax, turnEnd, turnAccel, turnDecel)*turnDir);
    }

    public void frr(double xDist, double xStart, double xMax, double xAccel, double xDecel, int xDir,
                    double yDist, double yStart, double yMax, double yAccel, double yDecel, int yDir,
                    double turnDist, double turnStart, double turnMax, double turnAccel, double turnDecel, int turnDir) {

        xDist = xDist / inchesPerSecAtMax;
        yDist = yDist / inchesPerSecAtMax;
        turnDist = turnDist;

        double xEnd = 0;
        double yEnd = 0;
        double turnEnd = 0;
        double k_x = Math.pow(xAccel, 2) + Math.pow(xDecel, 2);
        double k_y = Math.pow(yAccel, 2) + Math.pow(yDecel, 2);
        double k_turn = Math.pow(turnAccel, 2) + Math.pow(turnDecel, 2);
        if (xDist > (2/3.0)*Math.pow(xMax, 3)*k_x) {
            xEnd = (xDist/xMax) + (1/3.0)*Math.pow(xMax, 2)*k_x;
        } else {
            xEnd = Math.pow(3*xDist/2.0, (2/3.0))*Math.pow(k_x, (1/3.0));
        }
        xEnd += xStart;
        if (yDist > (2/3.0)*Math.pow(yMax, 3)*k_y) {
            yEnd = (yDist/yMax) + (1/3.0)*Math.pow(yMax, 2)*k_y;
        } else {
            yEnd = Math.pow(3*yDist/2.0, (2/3.0))*Math.pow(k_y, (1/3.0));
        }
        yEnd += yStart;
        if (turnDist > (2/3.0)*Math.pow(turnMax, 3)*k_turn) {
            turnEnd = (turnDist/turnMax) + (1/3.0)*Math.pow(turnMax, 2)*k_turn;
        } else {
            turnEnd = Math.pow(3*turnDist/2.0, (2/3.0))*Math.pow(k_turn, (1/3.0));
        }
        turnEnd += turnStart;
        double maxTime = Math.max(Math.max(xEnd, yEnd), turnEnd);
        ElapsedTime runtime = new ElapsedTime();
        double time = 0;
        while (time <= maxTime) {
            time = runtime.seconds() - 0.2;
            drive(celerate(time, xStart, xMax, xEnd, xAccel, xDecel)*xDir,
                  celerate(time, yStart, yMax, yEnd, yAccel, yDecel)*(-yDir),
                  celerate(time, turnStart, turnMax, turnEnd, turnAccel, turnDecel)*turnDir);
        }
    }

    public void frrNormie(double xDist, double xStart, int xDir,
                          double yDist, double yStart, int yDir,
                          double turnDist, double turnStart, int turnDir) {
        frr(xDist, xStart, 1, 1, 1, xDir,
            yDist, yStart, 1, 1, 1, yDir,
            turnDist, turnStart, 1, 1, 1, turnDir);
    }

    public boolean frrToBeUsedInALoop(double currentTime,
                                      double xDist, double xStart, double xMax, double xAccel, double xDecel, int xDir,
                                      double yDist, double yStart, double yMax, double yAccel, double yDecel, int yDir,
                                      double turnDist, double turnStart, double turnMax, double turnAccel, double turnDecel, int turnDir) {

        xDist = xDist / inchesPerSecAtMax;
        yDist = yDist / inchesPerSecAtMax;
        turnDist = turnDist;

        double xEnd = 0;
        double yEnd = 0;
        double turnEnd = 0;
        double k_x = Math.pow(xAccel, 2) + Math.pow(xDecel, 2);
        double k_y = Math.pow(yAccel, 2) + Math.pow(yDecel, 2);
        double k_turn = Math.pow(turnAccel, 2) + Math.pow(turnDecel, 2);
        if (xDist > (2/3.0)*Math.pow(xMax, 3)*k_x) {
            xEnd = (xDist/xMax) + (1/3.0)*Math.pow(xMax, 2)*k_x;
        } else {
            xEnd = Math.pow(3*xDist/2.0, (2/3.0))*Math.pow(k_x, (1/3.0));
        }
        xEnd += xStart;
        if (yDist > (2/3.0)*Math.pow(yMax, 3)*k_y) {
            yEnd = (yDist/yMax) + (1/3.0)*Math.pow(yMax, 2)*k_y;
        } else {
            yEnd = Math.pow(3*yDist/2.0, (2/3.0))*Math.pow(k_y, (1/3.0));
        }
        yEnd += yStart;
        if (turnDist > (2/3.0)*Math.pow(turnMax, 3)*k_turn) {
            turnEnd = (turnDist/turnMax) + (1/3.0)*Math.pow(turnMax, 2)*k_turn;
        } else {
            turnEnd = Math.pow(3*turnDist/2.0, (2/3.0))*Math.pow(k_turn, (1/3.0));
        }
        turnEnd += turnStart;
        double maxTime = Math.max(Math.max(xEnd, yEnd), turnEnd);
        if (currentTime > maxTime) { return false; }
        drive(celerate(currentTime, xStart, xMax, xEnd, xAccel, xDecel)*xDir,
              celerate(currentTime, yStart, yMax, yEnd, yAccel, yDecel)*(-yDir),
              celerate(currentTime, turnStart, turnMax, turnEnd, turnAccel, turnDecel)*turnDir);
        return true;
    }

    public boolean frrNormieLoop(double currentTime, double xDist, double xStart, int xDir,
                              double yDist, double yStart, int yDir,
                              double turnDist, double turnStart, int turnDir) {
        return frrToBeUsedInALoop(currentTime, xDist, xStart, 1, 1, 1, xDir,
                                  yDist, yStart, 1, 1, 1, yDir,
                                  turnDist, turnStart, 1, 1, 1, turnDir);
    }

    // what was this for again?
    public void mehRoadrunner(double time, double xEnd, int xDir, double yEnd, int yDir, double turnEnd, double turnDir) {
        drive(normie(time, xEnd)*xDir, normie(time, yEnd)*(-yDir), normie(time, turnEnd)*turnDir);
    }

    public void frrNormieByTime(double totalTime, double xEnd, int xDir, double yEnd, int yDir, double turnEnd, double turnDir) {
        ElapsedTime runtime = new ElapsedTime();
        while (runtime.seconds() < totalTime) {
            mehRoadrunner(runtime.seconds(), xEnd, xDir, yEnd, yDir, turnEnd, turnDir);
        }
    }

    // TeleOp Drive
    public void drive(double joystickX, double joystickY, double joystickTurn) {

        double x = joystickX;
        double y = joystickY;

        // Squared inputs mode
        if (isSquaredInputs) {

            // Store sign of value
            double signX;
            double signY;
            if (x < 0) signX = -1;
            else signX = 1;
            if (y < 0) signY = -1;
            else signY = 1;

            // Square the inputs
            x = Math.pow(x, 2) * signX;
            y = Math.pow(y, 2) * signY;

        }

        double turn = joystickTurn;
        if (reduceTurn) {
            // Reduce joystick turn
            turn = turn / 1.8;
        } else {
            turn = turn / 1.4;
        }

        // PID calculation
        pidTurn.setSetpoint(prevAngle);
        correction = pidTurn.performPID(gyro.getAngleDegrees());

        if (!thirdWheel) {

            // If turning or not moving then don't correct
            if (Math.abs(turn) != 0 || (x == 0 && y == 0)) {

                pidTurn.disable();
                pidTurn.reset();
                prevAngle = gyro.getAngleDegrees();
                correction = 0;

            } else pidTurn.enable();

        } else {

            // Continuous tuning
            if (Math.abs(turn) != 0) {

                pidTurn.disable();
                pidTurn.reset();
                prevAngle = gyro.getAngleDegrees();
                correction = 0;

            } else pidTurn.enable();

        }

        // If field oriented drive, then set angleCompensation to gyro angle
        double angleCompensation = 0;
        if (!isDrivePOV)
            angleCompensation = gyro.getAngleRadians();

        // Holonomic drive calculations

        // The magnitude of the joystick
        double r = Math.hypot(x, y);

        // The angle of the robot it is supposed to move towards
        double robotAngle = Math.atan2(-y, x) - Math.PI / 4 - angleCompensation;

        // Takes the x and y components of the vector
        double cosinePow = r * Math.cos(robotAngle);
        double sinePow = r * Math.sin(robotAngle);

        // Takes the bigger number and multiplies it by a ratio to speed up the robot
        double maxPow = Math.max(Math.abs(cosinePow), Math.abs(sinePow));
        double ratio = 0;
        if (maxPow != 0) ratio = r / maxPow;

        // Motor powers without PID corrections
        double v1 = cosinePow * ratio + turn;
        double v2 = sinePow * ratio - turn;
        double v3 = sinePow * ratio + turn;
        double v4 = cosinePow * ratio - turn;

        // Divide by ratio to make turning easier

        double flbrMax = Math.max(Math.abs(v1), Math.abs(v4));
        if (flbrMax > 1.0) {

            v1 /= flbrMax;
            v4 /= flbrMax;

        }

        double frblMax = Math.max(Math.abs(v2), Math.abs(v3));
        if (frblMax > 1.0) {

            v2 /= frblMax;
            v3 /= frblMax;

        }

        // Don't correct if isPID is false
        if (!isPID) correction = 0;
        //correction = 0;

        // Add PID corrections
        v1 -= correction;
        v2 += correction;
        v3 -= correction;
        v4 += correction;

        // Set motor powers
        double multiplier;
        if (isSlow) multiplier = SLOW_MULTIPLIER;
        else multiplier = 1;

        frontLeft.setPower(v1 * multiplier);
        frontRight.setPower(v2 * multiplier);
        backLeft.setPower(v3 * multiplier);
        backRight.setPower(v4 * multiplier);

        // Telemetry values
        //print("FLPow: ", frontLeft.getPower());
        //print("FRPow: ", frontRight.getPower());
        //print("BLPow: ", backLeft.getPower());
        //print("BRPow: ", backRight.getPower());
        print("Correction: ", correction);
        print("IsPOV: ", isDrivePOV);
        print("IsSlow: ", isSlow);
        print("Encoder fl: ", frontLeft.getCurrentPosition());
        print("Encoder fr: ", frontRight.getCurrentPosition());
        print("Encoder bl: ", backLeft.getCurrentPosition());
        print("Encoder br: ", backRight.getCurrentPosition());
        print("Gyro: ", gyro.getAngleDegrees());

    }

    public void setPowers(double fl, double fr, double bl, double br) {

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);

    }

    public void moveUntil(double speed, double angleMove, BoolCommand command) {

        // Reset motors and gyro
        gyro.reset();
        resetMotors();

        // For correction turning
        pidTurn.reset();
        pidTurn.setSetpoint(0);
        pidTurn.enable();

        // Calculate speed
        double flbrSpeed = speed * Math.sin((45.0 + angleMove) * Math.PI / 180);
        double frblSpeed = speed * Math.cos((45.0 + angleMove) * Math.PI / 180);

        // Maximize speed by calculating ratio
        double max = Math.max(Math.abs(flbrSpeed), Math.abs(frblSpeed));
        flbrSpeed = speed * (flbrSpeed / max);
        frblSpeed = speed * (frblSpeed / max);

        do {

            correction = pidTurn.performPID(gyro.getAngleDegrees());

            frontLeft.setPower(flbrSpeed - correction);
            frontRight.setPower(frblSpeed + correction);
            backLeft.setPower(frblSpeed - correction);
            backRight.setPower(flbrSpeed + correction);

            print("flbrP", flbrSpeed);
            print("frblP", frblSpeed);
            print("Boolean value: ", command.check());

            linearOpMode.telemetry.update();

        } while (linearOpMode.opModeIsActive() && !command.check());

        stop();

        linearOpMode.sleep(500);

    }

    public void move(double speed, double distance, double angleMove) {

        // The number of ticks the motors have to move without considering direction of motors
        int goalTicks = (int) ((distance / (wheelDiameter * Math.PI)) * ticksPerRev);

        // Calculate distance of motors
        int flbrDist = (int) (goalTicks * Math.sin(Math.toRadians(45.0 + angleMove)));
        int frblDist = (int) (goalTicks * Math.cos(Math.toRadians(45.0 + angleMove)));

        // Calculate speeds
        double max = Math.max(Math.abs(flbrDist), Math.abs(frblDist));
        double goalFLBRSpeed = speed * (flbrDist / max);
        double goalFRBLSpeed = speed * (frblDist / max);

        // For correction turning
        pidTurn.reset();
        pidTurn.setSetpoint(currAngle);
        pidTurn.enable();

        // For frontleft and backright motors
        pidFLBR.reset();
        pidFLBR.setInputRange(currFlbr, currFlbr + flbrDist);
        pidFLBR.setOutputRange(0, goalFLBRSpeed);
        pidFLBR.setSetpoint(currFlbr + flbrDist);
        pidFLBR.setTolerance(5);
        pidFLBR.enable();

        // For frontright and backleft motors
        pidFRBL.reset();
        pidFRBL.setInputRange(currFrbl, currFrbl + frblDist);
        pidFRBL.setOutputRange(0, goalFRBLSpeed);
        pidFRBL.setSetpoint(currFrbl + frblDist);
        pidFRBL.setTolerance(5);
        pidFRBL.enable();

        boolean isFLBROnTarget = false;
        boolean isFRBLOnTarget = false;
        if (flbrDist == 0) isFLBROnTarget = true;
        else if (frblDist == 0) isFRBLOnTarget = true;

        do {

            correction = pidTurn.performPID(gyro.getAngleDegrees());

            double avgFLBRPos = (frontLeft.getCurrentPosition() + backRight.getCurrentPosition()) / 2.0;
            double speedFLBR = pidFLBR.performPID(avgFLBRPos);

            double avgFRBLPos = (frontRight.getCurrentPosition() + backLeft.getCurrentPosition()) / 2.0;
            double speedFRBL = pidFRBL.performPID(avgFRBLPos);

            frontLeft.setPower(speedFLBR - correction);
            frontRight.setPower(speedFRBL + correction);
            backLeft.setPower(speedFRBL - correction);
            backRight.setPower(speedFLBR + correction);

            //print("Gyro: ", gyro.getAngleDegrees());
            print("FL: ", frontLeft.getCurrentPosition());
            print("FR: ", frontRight.getCurrentPosition());
            print("BL: ", backLeft.getCurrentPosition());
            print("BR: ", backRight.getCurrentPosition());
            //print("Correction: ", correction);
            print("FLBR Setpoint: ", pidFLBR.getSetpoint() + " " + Double.toString(flbrDist));
            print("FRBL Setpoint: ", pidFRBL.getSetpoint() + " " + Double.toString(frblDist));
            print("FLBR Speed: ", speedFLBR);
            print("FRBL Speed: ", speedFRBL);
            //print("FLBR Ontarget: ", pidFLBR.onTarget() || isFLBROnTarget);
            //print("FRBL Ontarget: ", pidFRBL.onTarget() || isFRBLOnTarget);

            linearOpMode.telemetry.update();

        } while (linearOpMode.opModeIsActive() && ((!pidFRBL.onTarget() && !isFRBLOnTarget) || (!pidFLBR.onTarget() && !isFLBROnTarget)));

        // Stop motors
        stop();

        // Add theoretical current position
        currFlbr += flbrDist;
        currFrbl += frblDist;

        // Add slight delay
        linearOpMode.sleep(500);

    }

    public void turn(double speed, int angle) {

        if (Math.abs(angle) > 359) angle = (int) Math.copySign(359, angle);

        pidAutoTurn.reset();
        pidAutoTurn.setInputRange(currAngle, currAngle - angle);
        pidAutoTurn.setOutputRange(0, speed);
        pidAutoTurn.setSetpoint(currAngle - angle);
        pidAutoTurn.setTolerance(5);
        pidAutoTurn.enable();

        do {

            speed = pidAutoTurn.performPID(gyro.getAngleDegrees());

            frontLeft.setPower(-speed);
            frontRight.setPower(speed);
            backLeft.setPower(-speed);
            backRight.setPower(speed);

            print("FL: ", frontLeft.getCurrentPosition());
            print("FR: ", frontRight.getCurrentPosition());
            print("BL: ", backLeft.getCurrentPosition());
            print("BR: ", backRight.getCurrentPosition());
            print("Curr Angle: ", gyro.getAngleDegrees());
            print("Setpoint: ", pidAutoTurn.getSetpoint());
            print("Is setpoint: ", pidAutoTurn.onTarget());

            linearOpMode.telemetry.update();

        } while (linearOpMode.opModeIsActive() && !pidAutoTurn.onTarget());

        // Add to current angle
        currAngle -= angle;

        // Stop motors
        stop();

        // Add slight delay
        linearOpMode.sleep(500);

    }

    public void stop() {

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }

    public void resetMotors() {

        stop();

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public double getVoltage(HardwareMap hwMap) {

        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hwMap.voltageSensor)
            if (sensor.getVoltage() > 0) result = Math.min(result, sensor.getVoltage());
        return result;

    }

}
