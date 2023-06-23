package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.LiftControlClass;

@Config
@TeleOp(name = "World's Best Teleop", group = "Competition")
public class WorldsBestTeleop extends OpMode {
    private PIDController controller;
    public static double kP=0;
    public static int tolerance=0;
    public static double p=0.0015, i=0, d=0;
    public static double f=-0.2;

    public static int target=0;

    private final double ticks_in_degrees = 8192/360;

    private int bumpCount;
    private int offset, liftPos;
    private boolean toggleReadyUp, toggleReadyDown, armOverride, alignAdjustReady, clawToggle, clawReady;
    private boolean armOverrideReady, armToggle;
    private HWProfile robot;
    private GamepadEx gp1;
    private LiftControlClass liftClass;
    private double strafePower, forwardPower, turnPower;
//    private opMode myOpmode= this;

    @Override
    public void init(){
        robot = new HWProfile();
        robot.init(hardwareMap);
        controller=new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        gp1 = new GamepadEx(gamepad1);
        telemetry.addData("Ready to Run: ", "GOOD LUCK");
        telemetry.update();

        liftClass = new LiftControlClass(robot);
        //lift.setPositionTolerance(tolerance);
        //lift.setPositionCoefficient(kP);
    }

    @Override
    public void loop(){
        if(robot.sensorColor.getDistance(DistanceUnit.CM)<3&&clawToggle){
            gamepad1.rumble(1,1,50);
        }

        //DRIVE CONTROL SECTION//
        //drive power input from analog sticks
        forwardPower=gp1.getLeftY();
        strafePower=gp1.getLeftX();
        if(gp1.getRightX()<0.75){
            turnPower=-Math.pow(gp1.getRightX(),2)*0.75*robot.TURN_MULTIPLIER;
        }else{
            turnPower=-gp1.getRightX()*robot.TURN_MULTIPLIER;
        }

        //FTCLib drive code, instantiated in HWProfile
        robot.mecanum.driveFieldCentric(strafePower,forwardPower,-gp1.getRightX()*robot.TURN_MULTIPLIER,robot.imu.getRotation2d().getDegrees()+180, true);        //END OF DRIVE CONTROL SECTION//

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
            liftClass.openClaw();
        } else if(!clawToggle){
            liftClass.closeClaw();
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
                liftClass.armScore();
            } else {
                liftClass.armIntake();
            }
        }else{
            liftClass.armIntake();
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
            target= robot.LIFT_BOTTOM;
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
            offset+=300;
        }

        if(!gp1.isDown(GamepadKeys.Button.X)){
            alignAdjustReady=true;
        }

        //apply lift positions
        if (bumpCount == 0) {
            liftPos = robot.LIFT_BOTTOM+offset;
            target = robot.LIFT_BOTTOM+offset;
        } else if (bumpCount == 1) {
            liftPos = robot.LIFT_LOW+offset;
            target = robot.LIFT_LOW+offset;
        } else if (bumpCount == 2) {
            liftPos = robot.LIFT_MID+offset;
            target = robot.LIFT_MID+offset;
        } else if (bumpCount == 3) {
            liftPos = robot.LIFT_HIGH+offset;
            target = robot.LIFT_HIGH+offset;
        }
        //adjust lift position
        if(gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0.5){
            offset-= robot.liftAdjust;
        }else if(gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0.5){
            offset += robot.liftAdjust;
        }

        //clip lift position to proper range (encoder counts in reverse so clip is negative)
        liftPos= Range.clip(liftPos,robot.MAX_LIFT_VALUE,-50);

        int armPos = robot.motorLiftLeft.getCurrentPosition();

        //FLIPPER CONTROL SECTION//
        if(gp1.isDown(GamepadKeys.Button.RIGHT_STICK_BUTTON)){
            liftClass.flippersDown();
        }else{
            liftClass.flippersUp();
        }
        //END OF FLIPPER CONTROL SECTION//

        controller.setPID(p,i,d);
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians((target/ticks_in_degrees)))*f;

        if(target>robot.LIFT_MID+1500){
            robot.lift.set(Range.clip(pid+ff,-0.7,0.7));
        }else {
            robot.lift.set(Range.clip(pid + ff, -1, 1));
        }

        telemetry.addData("pos: ",armPos);
        telemetry.addData("target: ",target);
    }

}
