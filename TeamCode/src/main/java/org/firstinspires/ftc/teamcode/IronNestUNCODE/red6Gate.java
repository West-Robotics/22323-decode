
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
    private Paths paths; // Paths defined in the Paths class
    private ElapsedTime gateHoldTimer;
    boolean gateHoldTimerUsed = false;


    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(-22.4,126.3, Math.toRadians(220)));
        follower.setMaxPower(0.85);
        timer = new ElapsedTime();
        gateHoldTimer = new ElapsedTime();

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

                                    new Pose(-44.5, 98)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(220))
                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(-44.5, 98),

                                    new Pose(-46, 88)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(-46, 88),

                                    new Pose(-14, 88)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(-14, 88),

                                    new Pose(-26, 88)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(-26, 88),

                                    new Pose(-26, 76.5)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(-26, 76.5),

                                    new Pose(-15, 74)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(-15, 74),

                                    new Pose(-26, 74)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path8 = follower.pathBuilder().addPath(

                            //PATH 8 IS NOT USED

                            new BezierLine(
                                    new Pose(-47.385, 74),

                                    new Pose(-47.385, 94)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();
            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(

                                    //MADE TO COME FROM PATH 7

                                    new Pose(-26, 74),

                                    new Pose(-44.5, 98)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(220))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(-44.5, 98),

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
                if(!follower.isBusy()){
                    // 1st Launch Here
                    launch(paths.Path2,2,0.925);
                }
                break;
            case 2:
                if(!follower.isBusy()){
                    In.setPower(-1);
                    Boost.setPower(-1);
                    follower.setMaxPower(0.65);
                    follower.followPath(paths.Path3);
                    setPathState(3);
                }
                break;
            case 3:
                //if it can't finish the path, just give up and do the next path
                if (!timerUsed){
                    timer.reset();
                    timerUsed = true;
                }

                if(!follower.isBusy() || timer.seconds()>3){
                    follower.breakFollowing();
                    follower.setMaxPower(0.75);
                    sleep(500);
                    In.setPower(0);
                    Boost.setPower(0);
                    timerUsed = false;
                    follower.followPath(paths.Path4);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    follower.followPath(paths.Path5);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()){
                    follower.followPath(paths.Path6);
                    setPathState(6);
                }
                break;
            case 6:
                //if it can't finish the path, just give up and do the next path
                if (!timerUsed){
                    timer.reset();
                    timerUsed = true;
                }
                if (timer.seconds()>3){
                    follower.followPath(paths.Path7);
                    setPathState(7);
                    timerUsed = false;
                }

                if(!follower.isBusy()){
                    //shorter wait here for sync
                    if (!gateHoldTimerUsed){
                        gateHoldTimer.reset();
                        gateHoldTimerUsed = true;
                    }

                }
                if(gateHoldTimerUsed && gateHoldTimer.seconds() > 3){
                    timerUsed = false;
                    gateHoldTimerUsed = false;
                    follower.followPath(paths.Path7);
                    setPathState(7);
                }
                break;
            case 7:

                //

                if(!follower.isBusy()){
                    follower.followPath(paths.Path9);
                    setPathState(9);
                }
                break;
            /* case 8:
                if(!follower.isBusy()){
                    follower.followPath(paths.Path9);
                    setPathState(9);
                }
                break; */
            case 9:
                if(!follower.isBusy()){
                    //follower.breakFollowing();
                    launch(paths.Path10,10,0.925);
                }
                break;
            case 10:
                setPathState(-1);
                break;
        }
    }
}
