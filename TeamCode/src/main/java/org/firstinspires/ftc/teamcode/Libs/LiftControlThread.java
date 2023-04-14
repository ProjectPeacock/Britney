package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;

public class LiftControlThread implements Runnable{


    //Odometry wheels
    private HWProfile tRobot = new HWProfile();
    private int armPos;
    private double pid;
    private double ff;
    private double liftPos;


    //Thead run condition
    private boolean isRunning = true;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime = 0;


    public LiftControlThread(HWProfile myRobot){
        this.tRobot = myRobot;
    }

    /**
     * Updates the global (x, y, theta) coordinate position of the robot using the odometry encoders
     */
    private void liftPositionUpdate(){
        armPos = tRobot.motorLiftLeft.getCurrentPosition();

        pid = tRobot.liftController.calculate(armPos, liftPos);
        ff = Math.cos(Math.toRadians((liftPos/tRobot.ticks_in_degrees)))*tRobot.kF;
        tRobot.lift.set(pid+ff);

    }

    public void setLiftPos(int targetPos){
        this.liftPos = targetPos;
    }
    public void shutdown(){ isRunning = false; }


    /**
     * Runs the thread
     */
    @Override
    public void run() {
        while(isRunning) {
            liftPositionUpdate();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
