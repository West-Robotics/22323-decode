package org.firstinspires.ftc.teamcode.IronNestUNCODE;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "immaHeadOut \uD83D\uDFE8", group = "Autonomous")

public class Leave extends Base_Robot_Auto{
    @Override
    public void init(){
init_motor();
    }
    public void loop(){
        leave();
    }
}


