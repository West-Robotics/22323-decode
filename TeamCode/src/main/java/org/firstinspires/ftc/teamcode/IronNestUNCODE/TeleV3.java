package org.firstinspires.ftc.teamcode.IronNestUNCODE;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.gamepad.GamepadManager;
import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@Configurable
@TeleOp(name = "Subsystem test")
public class TeleV3 extends Base_Robot{
    public TelemetryManager panelsTelemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        init_drivetrain();
        init_flywheels();
        setFlywheelVelocity(2630);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

         double leftFlywheelVelocity = OutL.getVelocity();
         double rightFlywheelVelocity = OutR.getVelocity();

        waitForStart();
        while (opModeIsActive()){
        panelsTelemetry.debug(leftFlywheelVelocity, rightFlywheelVelocity, getTargetFlywheelVelocity());
        panelsTelemetry.addData("left flywheel velocity readings",OutL.getVelocity());
        panelsTelemetry.addData("right flywheel velocity readings", OutR.getVelocity());
        panelsTelemetry.addData("Target velocity", getTargetFlywheelVelocity());
        moveFlywheels();
        moveRobot();
        panelsTelemetry.update(telemetry);
        }
    }
}
