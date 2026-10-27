package org.firstinspires.ftc.teamcode.IronNestBEE;

import static com.pedropathing.ivy.Scheduler.schedule;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.groups.Groups;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="shoot4Leave", group = "Autonomous")
public class shoot4Leave extends OpMode {
    private DriveTrain bee;
    private Launcher Ln;
    private Tollbooth Tb;
    private Intake In;
    private FlowerThingy FT;
    private Hood doomAndDespair;
    private Command autonomous(){
        return Command.build()
                .setExecute(()->{
                    Ln.spin();
                    try {
                        wait(2000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    In.spin();
                    Tb.open();

                }).requiring(bee, Ln, Tb, In);
    }
    @Override
    public void init() {
        Scheduler.reset();

        // 2. Initialize them here using hardwareMap
        bee = new DriveTrain(hardwareMap);
        Ln = new Launcher(hardwareMap);
        Tb = new Tollbooth(hardwareMap);
        In = new Intake(hardwareMap);
        FT = new FlowerThingy(hardwareMap);
        doomAndDespair = new Hood(hardwareMap);
    }
    @Override
    public void start(){

    }

    @Override
    public void loop(){}

}
