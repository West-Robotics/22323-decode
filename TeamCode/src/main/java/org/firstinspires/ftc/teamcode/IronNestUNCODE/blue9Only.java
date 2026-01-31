
package org.firstinspires.ftc.teamcode.IronNestUNCODE;
import static java.lang.Thread.sleep;

import com.pedropathing.paths.PathConstraints;
import com.pedropathing.util.Timer;
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

@Autonomous(name = "blue9Only", group = "Autonomous")
@Configurable // Panels
public class blue9Only extends Base_Robot_Auto {
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
        follower.setStartingPose(new Pose(22.4,126.3, Math.toRadians(320)));
        follower.setMaxPower(1);
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
        public PathChain Path8;
        public PathChain Path9;
        public  PathChain Path10;
        public  PathChain Path11;
        public  PathChain Path12;
        public  PathChain Path13;
        public  PathChain Path14;
        public  PathChain Path15;


        public Paths(Follower follower) {
            follower.setConstraints(new PathConstraints(0.995, 0.1, 0.75, 0.05, 100, 1, 10, 1));
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(22.4, 126.3),

                                    new Pose(49, 101)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(320), Math.toRadians(320))
                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(49, 101),

                                    new Pose(49, 88)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(49, 88),

                                    new Pose(14, 88)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(14, 88),

                                    new Pose(47.385, 88)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();


            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(47.385, 88),

                                    new Pose(47.385, 94)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();
            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(47.385, 94),

                                    new Pose(49, 101)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(320))

                    .build();

            //align for second pickup
            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(49, 101),

                                    new Pose(49, 63.5)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();
            //picking up second spike
            Path11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(49, 63.5),

                                    new Pose(10, 63.5)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();
            //leaving second spike
            Path12 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(10, 63.5),

                                    new Pose(49, 63.5)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();
            //going to launch position
            Path13 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(49, 63.5),

                                    new Pose(47.385, 94)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();
            //aligning
            Path14 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(47.385, 94),

                                    new Pose(49, 101)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(320))

                    .build();
            //leave
            Path15 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(49, 101),

                                    new Pose(49, 120)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(320))

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
                if(!follower.isBusy()){
                    In.setPower(-1);
                    follower.followPath(paths.Path11);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()){
                    follower.breakFollowing();
                    sleep(500);
                    In.setPower(0.2);
                    sleep(600);
                    In.setPower(0);
                    follower.followPath(paths.Path12);
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy()){
                    follower.followPath(paths.Path13);
                    setPathState(13);
                }
                break;
            case 13:
                if(!follower.isBusy()){
                    follower.followPath(paths.Path14);
                    setPathState(14);
                }
                break;
            case 14:
                if(!follower.isBusy()){
                    follower.breakFollowing();
                    launch(paths.Path15,15);
                }
                break;
            case 15:
                if(!follower.isBusy()){
                    follower.breakFollowing();
                }
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
    