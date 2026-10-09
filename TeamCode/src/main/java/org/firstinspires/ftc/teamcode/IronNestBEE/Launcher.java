package org.firstinspires.ftc.teamcode.IronNestBEE;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher {
    private final DcMotorEx Flywheel;
    private final Servo LaunchAdjust;

    /**
     * Initialize all things related to launching here
     *
     * @param hardwareMap The OpMode's hardware map.
     */
    public Launcher(HardwareMap hardwareMap) {
        Flywheel = hardwareMap.get(DcMotorEx.class, "Launcher");
        LaunchAdjust = hardwareMap.get(Servo.class, "Launch Adjust");
        Flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void spin() {
        Flywheel.setPower(-1);
    }
    public void stop(){
        Flywheel.setPower(0);
    }
    public Command Launch(Gamepad gamepad){
        return Command.build()
                .setExecute(()->{
                    if(gamepad.right_trigger>.2 || gamepad.left_trigger>.2) {
                        this.spin();
                    }else{
                        this.stop();
                    }
                    if(gamepad.a){
                        LaunchAdjust.setPosition(0.8);
                    }
                    if(gamepad.y){
                        LaunchAdjust.setPosition(0.1);
                    }
                        }).requiring(this);
    }
    public double getSpeed(){
       return Flywheel.getVelocity();
    }
    //Experimental, test with getSpeed to see if it works
    public void setSpeed(double speed){
        Flywheel.setVelocity(speed);
    }
}
