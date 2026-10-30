package org.firstinspires.ftc.teamcode.IronNestBEE;

import com.pedropathing.ivy.Command;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher {
    private final DcMotorEx Flywheel;
    private final Servo LaunchAdjust;
    private Timer timer;

    /**
     * Initialize all things related to launching here
     *
     * @param hardwareMap The OpMode's hardware map.
     */
    public Launcher(HardwareMap hardwareMap) {
        Flywheel = hardwareMap.get(DcMotorEx.class, "launcher");
        LaunchAdjust = hardwareMap.get(Servo.class, "launchAdjust");
        Flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        //LaunchAdjust.setPosition(0.95);
    }
    public void spin() {Flywheel.setPower(0.9);}
    public void stop(){
        Flywheel.setPower(0);
    }
    public void pollen(){
        LaunchAdjust.setPosition(0.95);
    }
    public Command Launch(Gamepad gamepad){
        return Command.build()
                .setExecute(()->{
                    if(gamepad.right_trigger>.2 || gamepad.x) {
                        this.spin();
                    }else{
                        this.stop();
                    }
                    if(gamepad.dpad_down){
                        //pollen position
                        LaunchAdjust.setPosition(0.95);
                    }
                    if(gamepad.dpad_up){
                        //nectar position
                        LaunchAdjust.setPosition(0.48);
                    }
                    if(gamepad.dpad_right){
                        //receiving position
                        LaunchAdjust.setPosition(0.22);
                    }
                        }).requiring(this);
    }
    public Command autoLaunch(){
        return Command.build()
                .setExecute(this::spin)
                .setEnd(endCondition -> this.stop())
                .requiring(this);
    }
    public double getSpeed(){
       return Flywheel.getVelocity();
    }
    //Experimental, test with getSpeed to see if it works
    public void setSpeed(double speed){
        Flywheel.setVelocity(speed);
    }
}
