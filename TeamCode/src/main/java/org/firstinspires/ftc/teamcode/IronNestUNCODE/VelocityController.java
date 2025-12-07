package org.firstinspires.ftc.teamcode.IronNestUNCODE;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.control.PIDController;


@Configurable
public class VelocityController {
    private  DcMotorEx motor;
    private final int DesiredVelocity;
    private final double agressiveness;
    private final double steadyErrorFixer;
    private final double decelerate;
    private final PIDController VelocityController;
    public VelocityController (int DesiredVelocity, double agressiveness, double steadyErrorFixer, double decelerate) {
        this.DesiredVelocity = DesiredVelocity;
        this.agressiveness = agressiveness;
        this.steadyErrorFixer = steadyErrorFixer;
        this.decelerate = decelerate;
        this.VelocityController = new PIDController(this.agressiveness, this.steadyErrorFixer, this.decelerate);
        VelocityController.enable();
        VelocityController.setSetpoint(this.DesiredVelocity);
        VelocityController.setOutputRange(0,1);

    }
    public void associateMotor(DcMotorEx motor){
        this.motor = motor;
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double performPID(){
        return VelocityController.performPID(this.motor.getVelocity());
    }
    public double performPID(double actualVelocity) {
        return VelocityController.performPID(actualVelocity);
    }

    public void WhatIsGoingOn(){
        telemetry.addData("Power sent by PID Controller", VelocityController.performPID(this.motor.getVelocity()));
        telemetry.update();
    }
    public void WhatIsGoingOn(DcMotorEx motor){
        telemetry.addData("Power sent by PID Controller", VelocityController.performPID(motor.getVelocity()));
        telemetry.update();

    }
}