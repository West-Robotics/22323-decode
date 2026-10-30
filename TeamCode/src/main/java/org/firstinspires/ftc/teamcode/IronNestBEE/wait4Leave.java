package org.firstinspires.ftc.teamcode.IronNestBEE;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
@Autonomous(name="wait4Leave", group = "Autonomous")
public class wait4Leave extends OpMode {
    private DriveTrain bee;
    private Launcher Ln;
    private Tollbooth TbL;
    private Tollbooth TbR;
    private Intake In;

    @Override
    public void init() {
        Scheduler.reset();

        // 2. Initialize them here using hardwareMap
        bee = new DriveTrain(hardwareMap);
        Ln = new Launcher(hardwareMap);
        TbL = new Tollbooth(hardwareMap,true);
        TbR = new Tollbooth(hardwareMap,false);
        In = new Intake(hardwareMap);
    }

    public Command waitS(double seconds) {
        Timer timer = new Timer();
        return Command.build()
                .setStart(timer::reset)
                .setDone(() -> timer.seconds() >= seconds);
    }
    @Override
    public void start(){
        final Command autonomous = sequential(
                waitS(18),
                bee.autoDrive(-0.4,0,0,0.75),
                race(
                        sequential(
                                waitS(1),
                                parallel(
                                        TbL.autoHold(3,true),
                                        TbR.autoHold(3,false)
                                )
                        ),
                        Ln.autoLaunch(),
                        In.autoTake()
                )
                        .then( bee.autoDrive(0,-0.4,0,1))
        );
        schedule(autonomous);
    }

    @Override
    public void loop(){Scheduler.execute();}
}
