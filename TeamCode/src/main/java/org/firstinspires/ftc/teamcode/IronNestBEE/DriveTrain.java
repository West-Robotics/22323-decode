package org.firstinspires.ftc.teamcode.IronNestBEE;

import com.pedropathing.ivy.Command;

//All things related to the driving and the drivetrain are made here
public class DriveTrain {

    public DriveTrain(){
        // Initialize all motors. This is called in the Init function of an Opmode

    }
    private void CalculatePower(){}
    /*TODO:
     write the Drivetrain method
     write get and set methods for each motor
     write the calculate motor method
     write the teleop command using calculatePower to send the power to the motors
     write what happens when the drive command executes
  */
    public Command Drive(){
        return Command.build()
                .setExecute(()->{})

                /* If the condition here happens, the robot will stop
                 listening to inputs **FOREVER**(until you restart the opmode) */
                .setDone(()->true)

                .setEnd(endCondition->{})
                .requiring(this);
        /*this command requires the Drivetrain, so other
         commands need to have any priority to happen. this can be useful for
         stopping it while something automated happens */
    }
}
