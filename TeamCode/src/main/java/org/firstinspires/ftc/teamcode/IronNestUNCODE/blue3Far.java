
package org.firstinspires.ftc.teamcode.IronNestUNCODE;

import com.pedropathing.paths.PathConstraints;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "blue3Far \uD83D\uDFE6", group = "Autonomous")
@Configurable // Panels
public class blue3Far extends Base_Robot_Auto {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    private Paths paths; // Paths defined in the Paths class
    boolean gateHoldTimerUsed = false;
    double waitTime=0;
    boolean isIncrementing = false;


    @Override
    public void init(){
        waitTime = 0;
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(59,9, Math.toRadians(90)));
        follower.setMaxPower(0.975);
        timer = new ElapsedTime();

        paths = new Paths(follower); // Build paths---
        setPathState(0);
    }
    public void init_loop() {
        paths = new blue3Far.Paths(follower); // Build paths---
        panelsTelemetry.debug("Current wait time selected: ",waitTime);
        panelsTelemetry.debug("Dpad Up +1s");
        panelsTelemetry.debug("Dpad Left for 20.5 seconds");
        panelsTelemetry.debug("Dpad Right for 10 seconds");
        panelsTelemetry.debug("Dpad Down -1s");
        panelsTelemetry.debug("Left Bumper for 0 seconds");
        panelsTelemetry.update(telemetry);
        if (gamepad1.dpad_up  && !isIncrementing) {
            waitTime +=1;
            isIncrementing = true;
        }
        if (gamepad1.dpad_down && !isIncrementing)
            waitTime -=1;
        isIncrementing = true;
        if (gamepad1.dpad_left)
            waitTime = 20.5;
        if (gamepad1.dpad_right)
            waitTime = 5;
        if(gamepad1.left_bumper)
            waitTime =0;
        if(!gamepad1.dpad_down && !gamepad1.dpad_up)
            isIncrementing = false;
        if(waitTime<0)
            waitTime = 0;

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

        public Paths(Follower follower) {
            follower.setConstraints(new PathConstraints(0.995, 0.1, 0.75, 0.05, 100, 1, 10, 1));
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(59, 9),

                                    new Pose(59, 120)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(59, 120),

                                    new Pose(59, 132)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

        }
    }


    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {
            case 0:
                if(!timerUsed)
                {
                    timer.reset();
                    timerUsed=true;
                }
                if(timer.seconds()>waitTime){
                    follower.followPath(paths.Path1);
                    setPathState(1);
                    timerUsed=false;
                }
                break;
            case 1:
                if(!follower.isBusy()){
                    follower.setMaxPower(0.85);
                    follower.followPath(paths.Path2);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()){
                    launch(paths.Path2,3,0.89);
                }
                break;
            case 3:
                setPathState(-1);
                follower.breakFollowing();
                break;
        }
    }
}
    