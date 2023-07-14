package org.firstinspires.ftc.teamcode.OpModes;


import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.LiftControlClass;

import java.util.List;

@TeleOp(name = "Teleop Layout 1", group = "Competition")
@Disabled
//@Disabled
public class TeleopLayout1 extends LinearOpMode {
    private final static HWProfile robot = new HWProfile();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        LinearOpMode myOpmode= this;
        LiftControlClass lift = new LiftControlClass(robot);

        GamepadEx gp1 = new GamepadEx(gamepad1);

        telemetry.addData("Ready to Run: ", "GOOD LUCK");
        telemetry.update();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        ElapsedTime runTime = new ElapsedTime();

        boolean clawToggle=false, clawReady=false, toggleReadyUp=false, toggleReadyDown=false, alignAdjustReady=false, armToggle=false, armOverrideReady = false, armOverride=false;
        boolean antiTip=true;
        double forwardPower=0, strafePower=0, turnPower=0;

        int liftPos=0, bumpCount=0,offset=0;

        waitForStart();
        robot.servoArm.setPosition(robot.SERVO_ARM_INTAKE);
        double startTilt=robot.imu.getAngles()[robot.ANTI_TIP_AXIS], currentTilt=0, tip=0;

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        int armPos = 0;
        double pid = 0, ff = 0;

        robot.lift.set(pid+ff);


        runTime.reset();

        while (opModeIsActive()) {

            //DRIVE CONTROL SECTION//
            //drive power input from analog sticks
            forwardPower=gp1.getLeftY();
            strafePower=gp1.getLeftX();
            if(gp1.getRightX()<0.75){
                turnPower=Math.pow(gp1.getRightX(),2)*0.75*robot.TURN_MULTIPLIER;
            }else{
                turnPower=gp1.getRightX()*robot.TURN_MULTIPLIER;
            }

            /**
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
             **/

            //FTCLib drive code, instantiated in HWProfile
            if(robot.fieldCentric){
                //field centric setup
                robot.mecanum.driveFieldCentric(strafePower,forwardPower,-gp1.getRightX()*robot.TURN_MULTIPLIER,robot.imu.getRotation2d().getDegrees()+180, true);
            }else{
                //robot centric setup
                robot.mecanum.driveRobotCentric(strafePower,forwardPower,-turnPower, true);
            }
            //END OF DRIVE CONTROL SECTION//


            //CLAW CONTROL SECTION//
            if(gp1.isDown(GamepadKeys.Button.A)&&clawReady){
                clawToggle=!clawToggle;
            }

            //forces claw to only open or close if button is pressed once, not held
            if(!gp1.isDown(GamepadKeys.Button.A)){
                clawReady=true;
            }else{
                clawReady=false;
            }

            //apply value to claw
            if (clawToggle) {
                lift.openClaw();
            } else if(!clawToggle){
                lift.closeClaw();
            }else if(gp1.isDown(GamepadKeys.Button.Y)){
                robot.servoGrabber.setPosition(robot.CLAW_BEACON);
            }
            //END OF CLAW CONTROL SECTION//

            //ARM CONTROL SECTION//
            if(gp1.isDown(GamepadKeys.Button.LEFT_STICK_BUTTON)&&armOverrideReady){
                armOverride=!armOverride;
            }

            //checks for arm button being held down
            if(!gp1.isDown(GamepadKeys.Button.LEFT_STICK_BUTTON)){
                armOverrideReady=true;
            }else{
                armOverrideReady=false;
            }

            //change armToggle based on lift position
            if(bumpCount>0){
                armToggle=true;
            }else{
                armToggle=false;
            }

            if (bumpCount==0){
                armOverride=false;
            }

            //apply value to arm
            if(!armOverride) {
                if (armToggle) {
                    lift.armScore();
                } else {
                    lift.armIntake();
                }
            }else{
                lift.armIntake();
            }
            //END OF ARM CONTROL SECTION//

            //LIFT CONTROL SECTION//
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
                armOverride=false;
            }

            //increase lift position
            if (gp1.getButton(GamepadKeys.Button.RIGHT_BUMPER)&&toggleReadyUp){
                offset=0;
                toggleReadyUp=false;
                if(bumpCount<3){
                    bumpCount++;
                }

            //decrease lift position
            }else if(gp1.getButton(GamepadKeys.Button.LEFT_BUMPER)&&toggleReadyDown){
                offset=0;
                toggleReadyDown=false;
                if(bumpCount>0){
                    bumpCount--;
                }
            }

            //move lift down to score
            if(gp1.isDown(GamepadKeys.Button.X)&&alignAdjustReady){
                offset+=800;
            }

            if(!gp1.isDown(GamepadKeys.Button.X)){
                alignAdjustReady=true;
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
                offset-= robot.liftAdjust;
            }else if(gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0.5){
                offset += robot.liftAdjust;
            }

            //clip lift position to proper range (encoder counts in reverse so clip is negative)
            liftPos=Range.clip(liftPos,robot.MAX_LIFT_VALUE,-50);

            //set lift target and run PID
            armPos = robot.motorLiftLeft.getCurrentPosition();

            pid = robot.liftController.calculate(armPos, liftPos);
            ff = Math.cos(Math.toRadians((liftPos/robot.ticks_in_degrees)))*robot.kF;

            robot.lift.set(pid+ff);
            //END OF LIFT CONTROL SECTION//

            //FLIPPER CONTROL SECTION//
            if(gp1.isDown(GamepadKeys.Button.Y)){
                lift.flippersDown();
            }else{
                lift.flippersUp();
            }
            //END OF FLIPPER CONTROL SECTION//


            //DRIVER FEEDBACK SECTION//
            //rumble if cone detected in claw AND if claw is open
            if(robot.sensorColor.getDistance(DistanceUnit.CM)<3&&clawToggle){
                gamepad1.rumble(1,1,50);
            }

            //Rumble controller for endgame and flash controller light red
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
            if(runTime.time() > 93) {gamepad1.setLedColor(255, 0, 0, 30000);
            }
            //END OF DRIVER FEEDBACK SECTION//

            //TELEMETRY//
            //telemetry.addData("lift position = ", robot.liftEncoder.getPosition());
            telemetry.addData("Lift Target Position = ", liftPos);
            telemetry.addData("Lift Position = ", robot.lift.getPositions().get(0));
            telemetry.addData("Arm Target Postion =", robot.servoArm.getPosition());
            telemetry.addData("Claw open = ", clawToggle);
            telemetry.addData("Current tip = ",tip);
            telemetry.addData("IMU Angles X = ", robot.imu.getAngles()[0]);
            telemetry.addData("IMU Angles Y = ", robot.imu.getAngles()[1]);
            telemetry.addData("IMU Angles Z = ", robot.imu.getAngles()[2]);
            telemetry.update();
            //END OF TELEMETRY//

        }   // end of while(opModeIsActive)
    }   // end of runOpMode()
}