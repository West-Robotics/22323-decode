package org.firstinspires.ftc.teamcode.IronNestUNCODE;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Configurable
@TeleOp(name = "TeleV3", group = "TeleOp")
public class TeleV3 extends Base_Robot{
    public TelemetryManager panelsTelemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        init_motor();
        init_flywheels();
        setFlywheelVelocity(2630);

        init_vision();
        if(USE_WEBCAM)
            setManualExposure();


        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

         double leftFlywheelVelocity = OutL.getVelocity();
         double rightFlywheelVelocity = OutR.getVelocity();

        waitForStart();

        while (opModeIsActive()){

        controlFlywheels();
        manageIntake();
        manage_servos();
        moveRobot();
        approachApriltags();
        panelsTelemetry.debug("Apriltag detected:" + lookForAprilTags());
        panelsTelemetry.update(telemetry);
        telemetry.update();
        }
    }
}
