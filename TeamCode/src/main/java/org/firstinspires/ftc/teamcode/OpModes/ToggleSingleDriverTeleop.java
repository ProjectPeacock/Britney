package org.firstinspires.ftc.teamcode.OpModes;


import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.LiftControlClass;

import java.util.List;

@TeleOp(name = "Super Secret Teleop, DO NOT RUN!", group = "Competition")
//@Disabled
public class ToggleSingleDriverTeleop extends LinearOpMode {
    private final static HWProfile robot = new HWProfile();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        LinearOpMode myOpmode= this;
        LiftControlClass lift = new LiftControlClass(robot,myOpmode);

        GamepadEx gp1 = new GamepadEx(gamepad1);
        ButtonReader aReader = new ButtonReader(gp1, GamepadKeys.Button.A);
        ButtonReader bReader = new ButtonReader(gp1, GamepadKeys.Button.DPAD_RIGHT);

        telemetry.addData("Ready to Run: ", "GOOD LUCK");
        telemetry.update();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        ElapsedTime runTime = new ElapsedTime();

        boolean clawToggle=false, clawReady=false, slowToggle=false, slowReady=false, toggleReadyUp=false, toggleReadyDown=false, alignAdjustReady=false, alignerToggle=false;
        boolean antiTip=true;
        double forwardPower=0, strafePower=0, turnPower=0;

        int liftPos=0, bumpCount=0,offset=0;


        waitForStart();
        robot.servoAlign.setPosition(robot.SERVO_ALIGN_UP);
        double startTilt=robot.imu.getAngles()[robot.ANTI_TIP_AXIS], currentTilt=0, tip=0;

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        runTime.reset();

        while (opModeIsActive()) {
            //rumble if cone detected in claw AND if claw is open
            if(robot.sensorColor.getDistance(DistanceUnit.CM)<3&&clawToggle){
                gamepad1.rumble(1,1,50);
            }

            //drive power input from analog sticks
            forwardPower=gp1.getLeftY();
            strafePower=gp1.getLeftX();
            if(gp1.getRightX()<0.75){
                turnPower=Math.pow(gp1.getRightX(),2)*0.75*robot.TURN_MULTIPLIER;
            }else{
                turnPower=gp1.getRightX()*robot.TURN_MULTIPLIER;
            }

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

            /*
            //toggle for slow mode
            if(bReader.isDown()&&slowReady){
                slowToggle=!slowToggle;
            }

            if(!bReader.isDown()){
                slowReady=true;
            }else{
                slowReady=false;
            }

             */

            //apply slow mode
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
                robot.mecanum.driveRobotCentric(strafePower,forwardPower,-turnPower, true);
            }


            //claw control
            if(aReader.isDown()&&clawReady){
                clawToggle=!clawToggle;
            }

            //forces claw to only open or close if button is pressed once, not held
            if(!aReader.isDown()){
                clawReady=true;
            }else{
                clawReady=false;
            }

            //apply value to claw
            if (clawToggle) {
                //robot.servoAlign.setPosition(robot.SERVO_ALIGN_UP);
                lift.openClaw();
            } else if(!clawToggle){
                lift.closeClaw();
            }else if(gp1.isDown(GamepadKeys.Button.Y)){
                robot.servoGrabber.setPosition(robot.CLAW_BEACON);
            }

            //lift toggles
            if(!gp1.isDown(GamepadKeys.Button.RIGHT_BUMPER)){
                toggleReadyUp=true;
            }
            if(!gp1.isDown(GamepadKeys.Button.LEFT_BUMPER)){
                toggleReadyDown=true;
            }

            //lift reset
            if(gp1.isDown(GamepadKeys.Button.B)){
                bumpCount=0;
                offset=0;
                liftPos= robot.LIFT_BOTTOM;
            }

            //increase lift position
            if (gp1.getButton(GamepadKeys.Button.RIGHT_BUMPER)&&toggleReadyUp){
                offset=0;
                toggleReadyUp=false;
                if(bumpCount<3){
                    bumpCount++;
                }

            //increase lift position
            }else if(gp1.getButton(GamepadKeys.Button.LEFT_BUMPER)&&toggleReadyDown){
                offset=0;
                toggleReadyDown=false;
                if(bumpCount>0){
                    bumpCount--;
                }
            }

            if(!gp1.isDown(GamepadKeys.Button.X)){
                alignAdjustReady=true;
            }

            if(gp1.isDown(GamepadKeys.Button.X)&&alignAdjustReady){
                offset-=75;
            }

            //apply lift positions
            if (bumpCount == 0) {
                liftPos = robot.LIFT_BOTTOM+offset;
            } else if (bumpCount == 1) {
                liftPos = robot.LIFT_LOW+offset;
            } else if (bumpCount == 2) {
                liftPos = robot.LIFT_MID+offset;
            } else if (bumpCount == 3) {
                liftPos = robot.LIFT_HIGH+offset;
            }

            //adjust lift position
            if(gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0.5){
                offset+= robot.liftAdjust;
            }else if(gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0.5){
                offset-= robot.liftAdjust;
            }

            liftPos=Range.clip(liftPos,2,robot.MAX_LIFT_VALUE);


            lift.runTo(liftPos);

            if(!gp1.isDown(GamepadKeys.Button.Y)) {
                if (robot.motorLiftFront.getCurrentPosition() > robot.ALIGNER_UP_THRESHOLD && !clawToggle) {
                    robot.servoAlign.setPosition(robot.SERVO_ALIGN_DOWN);
                }
            }else{
                robot.servoAlign.setPosition(robot.SERVO_ALIGN_UP);
            }

            if(runTime.time() > 90&&runTime.time()<90.25){
                gamepad1.rumble(50);
                gamepad1.setLedColor(255,0,0,50);
            }

            if(runTime.time() > 91&&runTime.time()<91.25){
                gamepad1.rumble(50);
                gamepad1.setLedColor(255,0,0,50);
            }

            if(runTime.time() > 92&&runTime.time()<92.25){
                gamepad1.rumble(50);
                gamepad1.setLedColor(255,0,0,50);
            }
            if(runTime.time() > 93) {
                gamepad1.setLedColor(255, 0, 0, 30000);
            }

            // Provide user feedback
            //telemetry.addData("lift position = ", robot.liftEncoder.getPosition());
            telemetry.addData("Lift Position = ", liftPos);
            telemetry.addData("Lift power = ",robot.motorLiftFront.getPower());
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