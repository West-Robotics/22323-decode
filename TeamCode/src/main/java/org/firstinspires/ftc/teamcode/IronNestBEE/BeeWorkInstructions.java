package org.firstinspires.ftc.teamcode.IronNestBEE;

import static com.pedropathing.ivy.Scheduler.schedule;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BetoWork", group = "TeleOp")
@Configurable
public class BeeWorkInstructions extends OpMode {
    private DriveTrain bee;
    private Launcher Ln;
    private Intake In;
    private Tollbooth TbR;
    private Tollbooth TbL;
    private FlowerThingy FT;
    private Hood doomAndDespair;
    @Override
    public void init() {
        Scheduler.reset();

        // 2. Initialize them here using hardwareMap
        bee = new DriveTrain(hardwareMap);
        Ln = new Launcher(hardwareMap);
        TbR = new Tollbooth(hardwareMap, false);
        TbL = new Tollbooth(hardwareMap,true);
        In = new Intake(hardwareMap);
        FT = new FlowerThingy(hardwareMap);
        doomAndDespair = new Hood(hardwareMap);
    }

    @Override
    public void start() {
        FT.up();
        TbR.close(false);
        TbL.close(true);
        Ln.pollen();
        schedule(
                Groups.loop(
                        bee.Drive(gamepad1)
                                .with(Ln.Launch(gamepad1))
                                .with(TbR.Hold(gamepad1,false))
                                .with(TbL.Hold(gamepad1,true))
                                .with(In.IntakeIt(gamepad1))
                                .with(FT.upDown(gamepad1))
                                .with(doomAndDespair.block(gamepad1))
                )
        );
    }

    @Override
    public void loop() {
        Scheduler.execute();
    }
}