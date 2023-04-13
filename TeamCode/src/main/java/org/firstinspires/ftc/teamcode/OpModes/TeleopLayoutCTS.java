package org.firstinspires.ftc.teamcode.OpModes;


import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.LiftControlClass;

import java.util.List;

@TeleOp(name = "World's Best Teleop CTS", group = "Competition")
//@Disabled
public class TeleopLayoutCTS extends LinearOpMode {
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

            //clip lift position to proper range (encoder counts in reverse so clip is negative)
            liftPos=Range.clip(liftPos,robot.MAX_LIFT_VALUE,-50);

            //set lift target and run PID
            armPos = robot.motorLiftLeft.getCurrentPosition();

            pid = robot.liftController.calculate(armPos, liftPos);
            ff = Math.cos(Math.toRadians((liftPos/robot.ticks_in_degrees)))*robot.kF;

            robot.lift.set(pid+ff);
            //END OF LIFT CONTROL SECTION//

        }   // end of while(opModeIsActive)
    }   // end of runOpMode()
}