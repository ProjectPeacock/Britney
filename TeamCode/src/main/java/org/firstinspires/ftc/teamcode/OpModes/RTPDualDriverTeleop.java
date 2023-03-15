package org.firstinspires.ftc.teamcode.OpModes;


import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.LiftControlClass;

import java.util.List;

@TeleOp(name = "RTP Dual Driver Teleop Mode", group = "Competition")
//@Disabled
public class RTPDualDriverTeleop extends LinearOpMode {
    private final static HWProfile robot = new HWProfile();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        LinearOpMode myOpmode= this;
        LiftControlClass lift = new LiftControlClass(robot,myOpmode);

        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);
        ButtonReader aReader = new ButtonReader(gp2, GamepadKeys.Button.LEFT_BUMPER);
        ButtonReader bReader = new ButtonReader(gp1, GamepadKeys.Button.RIGHT_BUMPER);

        ButtonReader aReader2 = new ButtonReader(gp1, GamepadKeys.Button.LEFT_BUMPER);

        telemetry.addData("Ready to Run: ", "GOOD LUCK");
        telemetry.update();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        boolean clawToggle=false, clawReady=false, slowToggle=false, slowReady=false;
        boolean antiTip=true;
        double forwardPower=0, strafePower=0, liftPower=.5;
        int liftPos=0;


        waitForStart();
        double startTilt=robot.imu.getAngles()[robot.ANTI_TIP_AXIS], currentTilt=0, tip=0;

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        while (opModeIsActive()) {
            forwardPower=gp1.getLeftY();
            strafePower=gp1.getLeftX();

            //anti-tip "algorithm"
            if(antiTip){
                currentTilt=robot.imu.getAngles()[robot.ANTI_TIP_AXIS];
                tip = Math.abs(currentTilt-startTilt);
                //if robot is tipped more than tolerance, multiply drive power by adjustment
                if(tip>robot.ANTI_TIP_TOL*2){
                    forwardPower*=-1;
                }else if(tip>robot.ANTI_TIP_TOL){
                    forwardPower*=robot.ANTI_TIP_ADJ;
                }
            }
            if(bReader.isDown()&&slowReady){
                slowToggle=!slowToggle;
            }

            if(!bReader.isDown()){
                slowReady=true;
            }else{
                slowReady=false;
            }
            if (slowToggle) {
                forwardPower*=0.5;
                strafePower*=0.5;
            } else {
                forwardPower*=1;
                strafePower*=1;
            }


            //mecanum drive setups
            if(robot.fieldCentric){
                //field centric setup
                robot.mecanum.driveFieldCentric(strafePower,forwardPower,-gp1.getRightX()*robot.TURN_MULTIPLIER,robot.imu.getRotation2d().getDegrees()+180, true);
            }else{
                //robot centric setup
                robot.mecanum.driveRobotCentric(strafePower,forwardPower,-gp1.getRightX()*robot.TURN_MULTIPLIER, true);
            }


            //claw control
            if((aReader.isDown()&&clawReady)||(aReader2.isDown()&&clawReady)){
                clawToggle=!clawToggle;
            }

            //forces claw to only open or close if button is pressed once, not held
            if(!aReader.isDown()&&!aReader2.isDown()){
                clawReady=true;
            }else{
                clawReady=false;
            }

            //apply value to claw
            if (clawToggle) {
                lift.openClaw();
            } else {
                lift.closeClaw();
            }


            if (gp2.getButton(GamepadKeys.Button.B)){
                liftPos= robot.LIFT_LOW;
            }else if(gp2.getButton(GamepadKeys.Button.Y)){
                liftPos= robot.LIFT_HIGH;
            }else if(gp2.getButton(GamepadKeys.Button.X)){
                liftPos= robot.LIFT_MID;
            }else if(gp2.getButton(GamepadKeys.Button.A)){
                liftPos= robot.LIFT_BOTTOM;
            }

            if(gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0.15){
                liftPos+= robot.liftAdjust;
            }else if(gp2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0.15){
                liftPos-= robot.liftAdjust;
            }

            if(liftPos<0){
                liftPos=0;
            }
            lift.runTo(liftPos);

            // Provide user feedback
            //telemetry.addData("lift position = ", robot.liftEncoder.getPosition());
            telemetry.addData("Lift Position = ", liftPos);
            telemetry.addData("Lift power = ",liftPower);
            telemetry.addData("Claw open = ", clawToggle);
            telemetry.addData("Current tip = ",tip);
            telemetry.addData("IMU Angles X = ", robot.imu.getAngles()[0]);
            telemetry.addData("IMU Angles Y = ", robot.imu.getAngles()[1]);
            telemetry.addData("IMU Angles Z = ", robot.imu.getAngles()[2]);
//            telemetry.addData("Inches traveled forward/backward ", robot.forwardBackwardOdo.getCurrentPosition()/(2*Math.PI*0.7480314960629921)); //taken from odometry, long number is wheel radius in inches
//            telemetry.addData("Inches traveled side/side ", robot.sideSideOdo.getCurrentPosition()/(2*Math.PI*0.7480314960629921));
            telemetry.update();

        }   // end of while(opModeIsActive)
    }   // end of runOpMode()
}