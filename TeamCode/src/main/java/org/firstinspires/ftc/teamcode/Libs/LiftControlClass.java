package org.firstinspires.ftc.teamcode.Libs;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;

public class LiftControlClass {

    private HWProfile robot;
    public OpMode opMode;
    public int cyclesRun=0;
    AutoParams params = new AutoParams();

    /*
     * Constructor method
     */

    public LiftControlClass(HWProfile myRobot, OpMode myOpMode){
        robot = myRobot;
        opMode = myOpMode;

    }   // close AutoClass constructor Method


    /**
     * Method: liftReset
     *  -   reset the lift to starting position
     */
    public void setLiftPow(double pow){
        robot.motorLiftLeft.setPower(pow);
        robot.motorLiftRight.setPower(pow);
    }
    public void runTo(int pos){
        setLiftPow(params.liftPow);
        robot.motorLiftLeft.setTargetPosition(pos);
        robot.motorLiftRight.setTargetPosition(pos);
        //robot.motorLiftRear.setTargetPosition(pos-params.diffConstant);
    }

    //method for moving lift to score and retract
    //pos corresponds to bottom, low, mid, high
    //0==bottom
    //1==low
    //2==mid
    //3==high
    public void moveLiftScore(int pos){
        if(pos==0){
            runTo(robot.LIFT_BOTTOM);
            robot.servoArm.setPosition(robot.SERVO_ARM_INTAKE);
        }else if(pos==1){
            runTo(robot.LIFT_LOW);
            robot.servoArm.setPosition(robot.SERVO_ARM_INTAKE);
        }else if(pos==2){
            runTo(robot.LIFT_MID);
            robot.servoArm.setPosition(robot.SERVO_ARM_SCORE);
        }else if(pos==3){
            runTo(robot.LIFT_HIGH);
            robot.servoArm.setPosition(robot.SERVO_ARM_SCORE);
        }
    }

    public void moveLiftScore(int pos, int offset){
        if(pos==1){
            runTo(robot.LIFT_LOW-offset);
        }else if(pos==2){
            runTo(robot.LIFT_MID-offset);
        }else if(pos==3){
            runTo(robot.LIFT_HIGH-offset);
        }
    }

    //method for moving lift to retrieve cones
    //cyclesRun corresponds to how many cycles have been completed. This class keeps track of how many cycles have been completed internally
    public void moveLiftGrab(){
        if(cyclesRun==0){
            runTo(params.cycle1);
        }else if(cyclesRun==1){
            runTo(params.cycle2);
        }else if(cyclesRun==2){
            runTo(params.cycle3);
        }else if(cyclesRun==3){
            runTo(params.cycle4);
        }else if(cyclesRun==4){
            runTo(params.cycle5);
        }
        cyclesRun++;
    }

    //claw control methods
    public void openClaw(){
        robot.servoGrabber.setPosition(robot.CLAW_OPEN);
        robot.servoArm.setPosition(robot.SERVO_ARM_INTAKE);
    }
    public void closeClaw(){
        robot.servoGrabber.setPosition(robot.CLAW_CLOSE);
    }
    public void lowerAligner(){robot.servoArm.setPosition(robot.SERVO_ARM_SCORE);}
    public void disableClaw(){robot.servoGrabber.setPwmDisable();}
}   // close the AutoClass class