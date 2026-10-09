package org.firstinspires.ftc.teamcode.IronNestBEE;
import static com.pedropathing.ivy.Scheduler.schedule;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.groups.Groups;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "IvyTest", group = "TeleOp")
@Configurable
public class BeeWorkInstructions extends OpMode {
    @Override
    public void init() {
        Scheduler.reset();
        DriveTrain bee = new DriveTrain(hardwareMap);
        // Schedule the default drive command to run in a loop
        schedule(
                Groups.loop(
                        bee.Drive(gamepad1)
                ));
    }

    @Override
    public void loop() {
        Scheduler.execute();
    }
}
