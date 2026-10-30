package org.firstinspires.ftc.teamcode.IronNestBEE;

import com.pedropathing.ivy.Command;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import javax.tools.Tool;

public class Tollbooth {
    private final Servo servo;
    private Timer timer;
    private boolean timerUsed;

    public Tollbooth(HardwareMap hardwareMap, boolean inverse){
        if(inverse) {
            servo = hardwareMap.get(Servo.class, "tollBoothL");
        }else{
            servo = hardwareMap.get(Servo.class, "tollBoothR");
        }
    }
    public void open(boolean inverse){
        if(inverse) {
            servo.setPosition(0.2);
        }else{
            servo.setPosition(0.575);
        }

    }
    public void close(boolean inverse){
        if(inverse) {
            servo.setPosition(0.525);
        }else{
            servo.setPosition(0.25);
        }
    }
    public Command Hold(Gamepad gamepad, boolean inverse){
        timer = new Timer();
        return Command.build()
                .setExecute(()->{
                    if(gamepad.a || gamepad.right_trigger>.2){

                        if(!timerUsed){
                            timer.reset();
                            timerUsed = true;
                        }
                        if(timer.seconds() >= 0.1 || gamepad.a) {
                            this.open(inverse);
                        }
//                        //Temporary until we can get timer working
//                        this.open(inverse);
                    }else{
                        this.close(inverse);
                        timerUsed = false;
                    }
                }).requiring(this);
    }
    public Command autoHold(double time, boolean inverse){
        timer = new Timer();
        return Command.build()
                .setStart(()->{timer.reset();})
                .setExecute(()->{this.open(inverse);})
                .setDone(()->timer.seconds() >= time)
                .setEnd(endCondition -> this.close(inverse))
                .requiring(this);
    }
}
