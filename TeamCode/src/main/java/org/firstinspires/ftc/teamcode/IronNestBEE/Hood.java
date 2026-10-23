package org.firstinspires.ftc.teamcode.IronNestBEE;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hood {
    public final Servo hood;
    public Hood(HardwareMap hardwareMap) {
        hood = hardwareMap.get(Servo.class, "hood");
    }
    public void up() {
        hood.setPosition(0.75);
    }
    public void down() {
        hood.setPosition(0);
    }
    public Command block(Gamepad gamepad) {
        return Command.build()
                .setExecute(() -> {
                    if (gamepad.left_bumper){
                        this.up();
                    } else if (gamepad.right_bumper) {
                        this.down();
                    }
                }).requiring(this);

    }

}
