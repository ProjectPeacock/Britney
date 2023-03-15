package org.firstinspires.ftc.teamcode.Libs;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;

public class LiftThread implements Runnable{

    private boolean isRunning = true;
    public static final double Kp = 0.002;;
    public static final double Kd = 0.0005;
    public int liftPosition = 0;
    public int differentialValue = 0;
    public double liftPower = 0;
    public double servoPos;
    public int cyclesRun = 0;
    public DcMotorEx motorFrontLift = null;
    public DcMotorEx motorRearLift = null;
    public ServoImplEx claw = null;
    public HWProfile robot = new HWProfile();
    public AutoParams threadParams = new AutoParams();

    public LiftThread(HWProfile myRobot, DcMotorEx motorLiftF, DcMotorEx motorLiftR, AutoParams params){
        this.robot = myRobot;
        this.claw = robot.servoGrabber;
        this.motorFrontLift = motorLiftF;
        this.motorRearLift = motorLiftR;
        this.threadParams = params;

    }

    public void moveLiftScore(int pos){
        if(pos==0){
            this.liftPosition= robot.LIFT_BOTTOM;
        }else if(pos==1){
            this.liftPosition= robot.LIFT_LOW;
        }else if(pos==2){
            this.liftPosition= robot.LIFT_MID;
        }else if(pos==3){
            this.liftPosition= robot.LIFT_HIGH;
        }
    }

    public void moveLiftGrab(){
        if(cyclesRun==0){
            this.liftPosition= threadParams.cycle1;
        }else if(cyclesRun==1){
            this.liftPosition= threadParams.cycle2;
        }else if(cyclesRun==2){
            this.liftPosition= threadParams.cycle3;
        }else if(cyclesRun==3){
            this.liftPosition= threadParams.cycle4;
        }else if(cyclesRun==4){
            this.liftPosition= threadParams.cycle5;
        }
        cyclesRun++;
    }


    //claw control methods
    public void openClaw(){
        this.servoPos=robot.CLAW_OPEN;
    }
    public void closeClaw(){
        this.servoPos=robot.CLAW_CLOSE;
    }


    public void update(){
        if ((liftPosition - motorFrontLift.getCurrentPosition()) > 0){
            if (motorFrontLift.getCurrentPosition() < liftPosition) {
                this.differentialValue = Math.abs(motorFrontLift.getCurrentPosition()- differentialValue);
                this.liftPower = 1;
                //this.liftPower = Math.abs(motorFrontLift.getCurrentPosition() - liftPosition) * this.Kp + (differentialValue * Kd);
                motorFrontLift.setPower(liftPower);
                motorRearLift.setPower(liftPower);
            } else if (motorFrontLift.getCurrentPosition() > liftPosition) {
                motorFrontLift.setPower(0);
                motorRearLift.setPower(0);
                this.differentialValue = 0;
            }
        } else if((liftPosition - motorFrontLift.getCurrentPosition()) < 0){
            if (motorFrontLift.getCurrentPosition() > liftPosition) {
                this.differentialValue = Math.abs(motorFrontLift.getCurrentPosition() - differentialValue);
                liftPower = 1;
                //                this.liftPower = (Math.abs(motorFrontLift.getCurrentPosition() - liftPosition) * Kp + (differentialValue * Kd));
//                if(liftPower < 0.25) liftPower = 0.25;
                motorFrontLift.setPower(-liftPower);
                motorRearLift.setPower(-liftPower);
            } else if (motorFrontLift.getCurrentPosition() < liftPosition) {
                motorFrontLift.setPower(0);
                motorRearLift.setPower(0);
                this.differentialValue = 0;
            }
        }
    }

    public void setTargetPosition(int targetPosition){
        this.liftPosition = targetPosition;
    }

    public void stopThread(){
        this.isRunning = false;
        this.motorRearLift.setPower(0);
        this.motorFrontLift.setPower(0);
    }


    public void run(){
        while(isRunning){
            update();
            try{
                Thread.sleep(100);
            } catch (InterruptedException e){
                e.printStackTrace();
            }   //end of try

        }       // end of while(isRunning)
    }   // end of method run()
}
