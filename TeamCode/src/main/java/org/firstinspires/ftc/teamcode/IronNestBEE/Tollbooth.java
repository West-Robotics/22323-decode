package org.firstinspires.ftc.teamcode.IronNestBEE;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import javax.tools.Tool;

public class Tollbooth {
    private final Servo servo;
    public Tollbooth(HardwareMap hardwareMap){
        servo = hardwareMap.get(Servo.class, "tollBoothL");
    }
    public void open(){
        servo.setPosition(0.2);
    }
    public void close(){
        servo.setPosition(0.575);
    }
    public Command Hold(Gamepad gamepad){
        return Command.build()
                .setExecute(()->{
                    if(gamepad.a){
                        this.open();
                    }else{
                        this.close();
                    }
                }).requiring(this);
    }
}
