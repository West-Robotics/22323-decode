package org.firstinspires.ftc.teamcode.IronNestUNCODE;
import static java.lang.Thread.sleep;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class pedroPathingTest extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState = 0; // Current autonomous path state (state machine)
    private Paths paths;
    public static double driveP, driveD, driveF,driveT;
    public static double strafeP, strafeD, strafeF;
    public static double headP, headD, headF;


    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(19, 120, Math.toRadians(320)));

        paths = new Paths(follower);
        driveD= follower.getConstants().coefficientsDrivePIDF.D;
        driveF= follower.getConstants().coefficientsDrivePIDF.F;
        driveP = follower.getConstants().coefficientsDrivePIDF.P;
        driveT = follower.getConstants().coefficientsDrivePIDF.T;

        strafeD = follower.getConstants().coefficientsTranslationalPIDF.D;
        strafeF = follower.getConstants().coefficientsTranslationalPIDF.F;
        strafeP = follower.getConstants().coefficientsTranslationalPIDF.P;

        headD = follower.getConstants().coefficientsHeadingPIDF.D;
        headF = follower.getConstants().coefficientsHeadingPIDF.F;
        headP = follower.getConstants().coefficientsHeadingPIDF.P;

        // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate();
        // Update autonomous state machine

        //Change PIDF values in panels
        follower.setDrivePIDFCoefficients(new FilteredPIDFCoefficients(driveP,0,driveD,driveT,driveF));
        follower.setTranslationalPIDFCoefficients(new PIDFCoefficients(strafeP,0,strafeD,strafeF));
        follower.setHeadingPIDFCoefficients(new PIDFCoefficients(headP,0,headD,headF));

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain MainChain;

        public Paths(Follower follower) {
            MainChain = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(19, 120), new Pose(47.385, 94.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(320), Math.toRadians(320))
                    .build();
        }
    }
    public void setPathState(int pState) {
        pathState = pState;
    }

    public int autonomousPathUpdate() {
        if(pathState==0){
            follower.followPath(paths.MainChain);
        }
        return 0;
    }
}