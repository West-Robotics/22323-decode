package org.firstinspires.ftc.teamcode.IronNestBEE;
import static com.pedropathing.ivy.Scheduler.schedule;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.groups.Groups;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "Betotype", group = "TeleOp")
@Configurable
public class BeePrototype extends OpMode {
    @Override
    public void init() {
        Scheduler.reset();
        Launcher beeLaunch = new Launcher(hardwareMap);
        // Schedule the default drive command to run in a loop
        schedule(
                Groups.loop(
                       beeLaunch.Launch(gamepad1)
                ));
    }

    @Override
    public void loop() {
        Scheduler.execute();
    }
}
