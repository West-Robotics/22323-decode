package org.firstinspires.ftc.teamcode.IronNestBEE;

import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name= "IvyTest")
@Configurable
/* This Code has for purpose to use Ivy to make a teleop demonstrating basic functions on a mecanum drivetrain*/
//TODO: Use commands from the Drivetrain to make a basic teleop
public class CommandsTest extends OpMode {
    @Override
    public void init (){
        Scheduler.reset();
        Command instructions = parallel(
               //What commands the robot does goes in here
        );
    }
    public void loop (){
        Scheduler.execute();
    }
}
