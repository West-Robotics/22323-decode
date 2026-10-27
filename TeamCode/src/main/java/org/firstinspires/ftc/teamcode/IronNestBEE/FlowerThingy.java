package org.firstinspires.ftc.teamcode.IronNestBEE;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FlowerThingy {
    private final Servo swivel;
    public FlowerThingy(HardwareMap hardwareMap){
        swivel = hardwareMap.get(Servo.class, "hockeyL");
    }
    public void up(){
        swivel.setPosition(0.75);
    }
    public void down(){
        swivel.setPosition(0);
    }
    public Command upDown(Gamepad gamepad){
        return Command.build()
                .setExecute(()->{
                    if(gamepad.b){
                        this.up();
                    } else if (gamepad.y) {
                        this.down();
                    }

                }).requiring(this);
    }
}
