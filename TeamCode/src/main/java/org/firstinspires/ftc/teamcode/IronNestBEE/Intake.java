package org.firstinspires.ftc.teamcode.IronNestBEE;

import com.pedropathing.ivy.Command;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
private final DcMotorEx Intake;
    private Timer timer;
public Intake(HardwareMap hardwareMap){
    Intake = hardwareMap.get(DcMotorEx.class, "intake");
}
public void spin(){
    Intake.setPower(-1);
}
public void stop(){
    Intake.setPower(0);
}
public void spinBack(){
    Intake.setPower(1);
}
public void spin(double power){
    Intake.setPower(power);
}
public double getSpeed(){
    return Intake.getVelocity();
}
public void setSpeed(double speed){
    Intake.setVelocity(speed);
}
public Command IntakeIt(Gamepad gamepad){
    return Command.build()
            .setExecute(()->{
                if(gamepad.left_trigger>.2 || gamepad.right_trigger>.2){
                    this.spin();
                }else{
                    this.stop();
                }
            }).requiring(this);
}

    public Command autoTake(double time){
        timer = new Timer();
        return Command.build()
                .setStart(()->{timer.reset();})
                .setExecute(this::spin)
                .setDone(()->timer.seconds() == time)
                .setEnd(endCondition -> this.stop())
                .requiring(this);
    }
}
