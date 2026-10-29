package org.firstinspires.ftc.teamcode.IronNestBEE;

import static com.pedropathing.ivy.Scheduler.schedule;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="shoot4Leave", group = "Autonomous")
public class shoot4Leave extends OpMode {
    private DriveTrain bee;
    private Launcher Ln;
    private Tollbooth TbL;
    private Tollbooth TbR;
    private Intake In;

    private Timer timer;
    public Command waiting(double time){
        timer = new Timer();
        return Command.build()
                .setStart(()->{timer.reset();})
                .setDone(()->timer.seconds() == time)
                .requiring(this);
    }
    private Command autonomous(){
        return Command.build()
                .setExecute(()->{
                    Ln.spin();
                    waiting(2);
                }).requiring(bee, Ln, TbL, In);
    }
    @Override
    public void init() {
        Scheduler.reset();

        // 2. Initialize them here using hardwareMap
        bee = new DriveTrain(hardwareMap);
        Ln = new Launcher(hardwareMap);
        TbL = new Tollbooth(hardwareMap);
        TbR = new Tollbooth(hardwareMap);
        In = new Intake(hardwareMap);
    }
    @Override
    public void start(){
        schedule(
                Groups.race(
                        Groups.loop(Ln.autoLaunch()).with(
                                Groups.parallel(
                                        In.autoTake(2)
                                        .with( TbL.autoHold(1 ,true))
                                        .with(In.autoTake(2)))
                                        .with(TbL.autoHold(2,true))
                                        .with(TbR.autoHold(2,false))
                        )
                ).then(
                        bee.autoDrive(-1,0,0,2)
                )
        );
    }

    @Override
    public void loop(){Scheduler.execute();}
}
