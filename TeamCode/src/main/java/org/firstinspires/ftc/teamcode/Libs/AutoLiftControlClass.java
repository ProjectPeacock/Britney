package org.firstinspires.ftc.teamcode.Libs;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;

public class AutoLiftControlClass {

    private HWProfile robot;
    public OpMode opMode;
    public int cyclesRun=0;
    AutoParams params = new AutoParams();


    /*
     * Constructor method
     */

    public AutoLiftControlClass(HWProfile myRobot, OpMode myOpMode){
        robot = myRobot;
        opMode = myOpMode;

    }   // close LiftControlClass constructor Method


    /**
     * Method: liftReset
     *  -   reset the lift to starting position
     */
    public void runTo(int target){
        int armPos = robot.motorLiftLeft.getCurrentPosition();

        double pid = robot.liftController.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians((target/robot.ticks_in_degrees)))*robot.kF;

        robot.lift.set(pid+ff);
    }

    //method for moving lift to score and retract
    //pos corresponds to bottom, low, mid, high
    //0==bottom
    //1==low
    //2==mid
    //3==high
    public int moveLiftScore(int pos, boolean armScore){
        if(armScore){
            armScore();
        }else{
            armIntake();
        }
        if(pos==0){
            return(robot.LIFT_BOTTOM);
        }else if(pos==1){
            return(robot.LIFT_LOW);
        }else if(pos==2){
            return(robot.LIFT_MID);
        }else{
            return(robot.LIFT_HIGH+250);
        }
    }

    public int moveLiftScore(int pos, int offset, boolean armScore){
        if(armScore){
            armScore();
        }else{
            armIntake();
        }
        if(pos==1){
            return(robot.LIFT_LOW-offset);
        }else if(pos==2){
            return(robot.LIFT_MID-offset);
        }else{
            return(robot.LIFT_HIGH-offset);
        }
    }

    //method for moving lift to retrieve cones
    //cyclesRun corresponds to how many cycles have been completed. This class keeps track of how many cycles have been completed internally
    public int moveLiftGrab(){
        armIntake();
        cyclesRun++;
        if(cyclesRun==0){
            return -13775;
        }else if(cyclesRun==1){
            return -9625;
        }else if(cyclesRun==2){
            return -7100;
        }else if(cyclesRun==3){
            return -4850;
        }else{
            return(robot.LIFT_BOTTOM);
        }
    }

    //mechanism control methods
    public void openClaw(){robot.servoGrabber.setPosition(robot.CLAW_OPEN);}
    public void closeClaw(){
        robot.servoGrabber.setPosition(robot.CLAW_CLOSE);
    }
    public void armIntake(){robot.servoArm.setPosition(robot.SERVO_ARM_INTAKE);}
    public void armScore(){robot.servoArm.setPosition(robot.SERVO_ARM_SCORE);}
    public void flippersUp(){robot.servoFlipperLeft.setPosition(robot.SERVO_FLIPPER_UP);robot.servoFlipperRight.setPosition(1-robot.SERVO_FLIPPER_UP);}
    public void flippersDown(){robot.servoFlipperLeft.setPosition(robot.SERVO_FLIPPER_DOWN);robot.servoFlipperRight.setPosition(1-robot.SERVO_FLIPPER_DOWN);}
    public void disableClaw(){robot.servoGrabber.setPwmDisable();}
}   // close the AutoClass class