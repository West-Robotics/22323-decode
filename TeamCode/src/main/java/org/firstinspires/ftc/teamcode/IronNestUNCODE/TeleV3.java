package org.firstinspires.ftc.teamcode.IronNestUNCODE;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Subsystem test")
public class TeleV3 extends Base_Robot{
    public TelemetryManager panelsTelemetry;
    private double leftFlywheelVelocity = OutL.getVelocity();
    private double rightFlywheelVelocity = OutR.getVelocity();

    @Override
    public void runOpMode() throws InterruptedException {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        init_drivetrain();
        init_flywheels();
        setFlywheelVelocity(2630);

        while (opModeIsActive()){
        panelsTelemetry.debug(leftFlywheelVelocity, rightFlywheelVelocity, getTargetFlywheelVelocity());
        moveFlywheels();
        panelsTelemetry.update(telemetry);
        }
    }
}
