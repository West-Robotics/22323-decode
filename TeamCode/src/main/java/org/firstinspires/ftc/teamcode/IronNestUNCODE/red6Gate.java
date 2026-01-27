
package org.firstinspires.ftc.teamcode.IronNestUNCODE;
import static java.lang.Thread.sleep;

import com.pedropathing.paths.PathConstraints;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "red6Gate", group = "Autonomous")
@Configurable // Panels
public class red6Gate extends Base_Robot_Auto {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Timer pathTimer, actionTimer, opmodeTimer; // Timer for autonomous paths
    private ElapsedTime timer;
    boolean timerUsed = false;
    private int iteration = 0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(-22.4,126.3, Math.toRadians(220)));
        follower.setMaxPower(0.75);
        timer = new ElapsedTime();

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        setPathState(0);
    }
    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        try {
            autonomousPathUpdate(); // Update autonomous state machine
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        init_motor();

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("is the timer being used?", timerUsed);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }


    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public  PathChain Path10;

        public Paths(Follower follower) {
            follower.setConstraints(new PathConstraints(0.995, 0.1, 0.75, 0.05, 100, 1, 10, 1));
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(-22.4, 126.3),

                                    new Pose(-49, 100)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(220))
                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(-49, 100),

                                    new Pose(-49, 84)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(-49, 84),

                                    new Pose(-14, 84)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(-14, 84),

                                    new Pose(-25.692, 84)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(-25.692, 84),

                                    new Pose(-25.821, 71.231)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(90))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(-25.821, 71.231),

                                    new Pose(-14.205, 71.385)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(-14.205, 71.385),

                                    new Pose(-47.385, 71.590)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(-47.385, 71.590),

                                    new Pose(-47.385, 94)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))

                    .build();
            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(-47.385, 94),

                                    new Pose(-49, 100)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(220))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(-49, 100),

                                    new Pose(-49, 120)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(220))

                    .build();
        }
    }


    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {
            case 0:
                follower.followPath(paths.Path1);
                setPathState(1);
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                /* Score Preload */

                /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                if(!follower.isBusy()){
                    follower.breakFollowing();
                    // 1st Launch Here
                    launch(paths.Path2,2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */

                /* Grab Sample */

                /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                if(!follower.isBusy()){
                    In.setPower(-1);
                    follower.setMaxPower(0.6);
                    follower.followPath(paths.Path3);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                /* Score Sample */

                /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                if(!follower.isBusy()){
                    follower.breakFollowing();
                    follower.setMaxPower(0.75);
                    sleep(500);
                    In.setPower(0.2);
                    sleep(600);
                    In.setPower(0);
                    follower.followPath(paths.Path4);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                /* Grab Sample */

                /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                if(!follower.isBusy()){
                    follower.followPath(paths.Path5);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */

                /* Score Sample */

                /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                if(!follower.isBusy()){
                    follower.followPath(paths.Path6);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                /* Grab Sample */

                /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                if(!follower.isBusy()){
                    follower.breakFollowing();
                    sleep(3000);
                    follower.followPath(paths.Path7);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                /* Grab Sample */

                /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                if(!follower.isBusy()){
                    follower.followPath(paths.Path8);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()){
                    follower.followPath(paths.Path9);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()){
                    follower.breakFollowing();
                    launch(paths.Path10,10);
                }
                break;
            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                setPathState(-1);
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
    }

    public void launch(PathChain path, int nextPath) throws InterruptedException{
        if (!timerUsed){
            timer.reset();
            iteration = 0;
            timerUsed = true;
        }
        sleep(200);
        if (timer.seconds()<1){;
            sleep(15);
            liftL.setPosition(0.01);
            liftR.setPosition(0.99);
            telemetry.addData("Status", "Outtake");
            OutL.setPower(0.95); OutR.setPower(0.95);
        }else {
            OutL.setPower(0.95); OutR.setPower(0.95);
            liftL.setPosition(0.22);
            liftR.setPosition(0.78);
            sleep(150);
            timer.reset();
            iteration += 1;
            telemetry.addData("Status", "Outtake Complete");
            if(iteration == 1) {
                In.setPower(-1);
            }
            if(iteration == 3) {
                OutL.setPower(0); OutR.setPower(0);
                In.setPower(0);
                timerUsed = false;
                follower.followPath(path);
                setPathState(nextPath);
            }
        }
    }
}
    