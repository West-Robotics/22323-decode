package org.firstinspires.ftc.teamcode.IronNestBEE;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Hood {
    private final ServoImplEx hood;

    public Hood(HardwareMap hardwareMap) {
        hood = hardwareMap.get(ServoImplEx.class, "hood");
        PwmControl.PwmRange range = new PwmControl.PwmRange(500, 2500);
        hood.setPwmEnable();
        hood.setPwmRange(range);
    }
    public void up() {
        hood.setPosition(0.8);
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
