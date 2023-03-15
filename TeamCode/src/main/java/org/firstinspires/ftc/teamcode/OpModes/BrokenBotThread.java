package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.LiftControlClass;
import org.firstinspires.ftc.teamcode.Libs.AutoParams;
import org.firstinspires.ftc.teamcode.Libs.LiftThread;

import java.util.List;

@Config
@TeleOp(name = "Broken Bot - Thread Testing", group = "Development")

public class BrokenBotThread extends LinearOpMode {
    private final static HWProfile robot = new HWProfile();
    FtcDashboard dashboard;
    public static double l01_CLAW_OPEN = robot.CLAW_OPEN;
    public static double l02_CLAW_CLOSE = robot.CLAW_CLOSE;
    public static int l03_LIFT_JUNCTION_HIGH = robot.LIFT_HIGH;
    public static int l04_LIFT_JUNCTION_MID = robot.LIFT_MID;
    public static int l05_LIFT_JUNCTION_LOW = robot.LIFT_LOW;
    public static int l06_LIFT_POSITION = 0;
    public static double l07_Lift_Up_Power = robot.LIFT_POW;
    public static double l08_Kp = 0.002;
    public static double l09_Ki = 0.00001;
    public static double l10_Kd = 0.0005;

    @Override
    public void runOpMode() {
        boolean fieldCentric = true;
        boolean liftToPosition = true;
        int liftPosition = 0;
        LinearOpMode opMode = this;
        DcMotorEx motorLiftF = null;
        DcMotorEx motorLiftR = null;
        double differentialValue = 0;

        //init params
        AutoParams params = new AutoParams();


        dashboard = FtcDashboard.getInstance();
        TelemetryPacket dashTelemetry = new TelemetryPacket();

        robot.init(hardwareMap);
        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);
        ButtonReader clawToggleButton = new ButtonReader(gp1, GamepadKeys.Button.RIGHT_BUMPER);

        LiftControlClass drive = new LiftControlClass(robot, opMode);

        motorLiftF = hardwareMap.get(DcMotorEx.class, "motorLiftFront");
        motorLiftF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorLiftF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLiftF.setPower(0);

        motorLiftR = hardwareMap.get(DcMotorEx.class, "motorLiftRear");
        motorLiftR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorLiftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLiftR.setPower(0);


        // start up the shooter control program which runs in parallel
        LiftThread liftControl = new LiftThread(robot, motorLiftF, motorLiftR, params);
        Thread lift = new Thread(liftControl);

        telemetry.addData("Ready to Run: ", "GOOD LUCK");
        telemetry.update();


        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        double liftPower=0;
        boolean clawToggle=false, clawReady=false;

        // post telemetry to FTC Dashboard as well
        dashTelemetry.put("01 - IMU Angle X = ", robot.imu.getAngles()[0]);
        dashTelemetry.put("02 - IMU Angle Y = ", robot.imu.getAngles()[1]);
        dashTelemetry.put("03 - IMU Angle Z = ", robot.imu.getAngles()[2]);
        dashTelemetry.put("04 - Lift Front Encoder Value = ", robot.motorLiftFront.getCurrentPosition());
        dashTelemetry.put("05 - Lift Rear Encoder Value = ", robot.motorLiftRear.getCurrentPosition());
        dashTelemetry.put("06 - Claw Value = ", robot.servoGrabber.getPosition());
        dashTelemetry.put("07 - GP1.Button.A = ", "RESET LIFT");
        dashTelemetry.put("08 - GP1.Button.B = ", "LIFT LOW JUNCTION");
        dashTelemetry.put("09 - GP1.Button.X = ", "LIFT MID JUNCTION");
        dashTelemetry.put("10 - GP1.Button.Y = ", "LIFT HIGH JUNCTION");
        dashTelemetry.put("11 - GP2.Button.A = ", "Custom Position - program stack cone levels");
        dashTelemetry.put("12 - Lift Power = ", liftPower);
        dashboard.sendTelemetryPacket(dashTelemetry);

        lift.start();
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad2.right_trigger > 0.1) {
                robot.autoLight.set(-1);
            } else {
                robot.autoLight.set(0);
            }
            if (fieldCentric) {
                robot.mecanum.driveFieldCentric(gp1.getLeftX(), gp1.getLeftY(), -gp1.getRightX() * robot.TURN_MULTIPLIER, robot.imu.getRotation2d().getDegrees() + 180, true);
            } else {
                robot.mecanum.driveRobotCentric(gp1.getLeftX(), gp1.getLeftY(), -gp1.getRightX() * robot.TURN_MULTIPLIER, true);
            }


            if(clawToggleButton.isDown()&&clawReady){
                clawToggle=!clawToggle;
            }
            if(!clawToggleButton.isDown()){
                clawReady=true;
            }else{
                clawReady=false;
            }
            if (clawToggle) {
                robot.servoGrabber.setPosition(robot.CLAW_OPEN);
            } else {
                robot.servoGrabber.setPosition(robot.CLAW_CLOSE);
            }



/*
            if(gp1.isDown(GamepadKeys.Button.DPAD_UP)) {
                robot.motorLF.set(1);
            } else if (gp1.isDown(GamepadKeys.Button.DPAD_DOWN)){
                robot.motorLR.set(1);
            } else if (gp1.isDown(GamepadKeys.Button.DPAD_LEFT)) {
                robot.motorRF.set(1);
            } else if (gp1.isDown(GamepadKeys.Button.DPAD_RIGHT)){
                robot.motorRR.set(1);
            }
            */

            /*
             * #############################################################
             * #################### LIFT CONTROL ###########################
             * #############################################################
             */

            if (gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1) {
                liftPosition = liftPosition - 10;
                liftPower = l07_Lift_Up_Power;

            } else if (gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1) {
                liftPosition = liftPosition + 10;
                liftPower = l07_Lift_Up_Power;
            } else {
            }

            if (gp1.isDown(GamepadKeys.Button.A)) {
                liftPosition = robot.LIFT_BOTTOM;
                liftPower = l07_Lift_Up_Power;
            }

            if (gp1.isDown(GamepadKeys.Button.B)) {
                liftPosition = l05_LIFT_JUNCTION_LOW;
                liftPower = l07_Lift_Up_Power;
            }

            if (gp1.isDown(GamepadKeys.Button.X)) {
                liftPosition = l04_LIFT_JUNCTION_MID;
                liftPower = l07_Lift_Up_Power;
            }

            if (gp1.isDown(GamepadKeys.Button.Y)) {
                liftPosition = l03_LIFT_JUNCTION_HIGH;
                liftPower = l07_Lift_Up_Power;
            }

            if (gp2.isDown(GamepadKeys.Button.A)) {
                liftPosition = l06_LIFT_POSITION;
                liftPower = l07_Lift_Up_Power;
            }

            liftPosition = Range.clip(liftPosition, robot.LIFT_BOTTOM, robot.MAX_LIFT_VALUE);

            liftControl.setTargetPosition(liftPosition);

//            robot.winch.setTargetPosition(liftPosition);
//            robot.winch.set(liftPower);




            /*
            motorLiftF.setTargetPosition(liftPosition);
            telemetry.addData("motorLiftF = ", motorLiftF.getCurrentPosition());
            motorLiftR.setVelocity(motorLiftF.getVelocity());
            telemetry.addData("motorLiftR = ", motorLiftR.getCurrentPosition());
            motorLiftF.setPower(liftPower);

             */


            /*
            if ((liftPosition - robot.motorLiftFront.getCurrentPosition()) > 0){
                telemetry.addData("UP: Distance to target position = ", (liftPosition - robot.motorLiftFront.getCurrentPosition()));
                if (robot.motorLiftFront.getCurrentPosition() < liftPosition) {
                    liftPower = Math.abs(robot.motorLiftFront.getCurrentPosition() - liftPosition) * l08_Kp;
                    robot.winch.set(liftPower);
                    telemetry.addData("UP: Lift Power : ", liftPower);
                } else if (robot.motorLiftFront.getCurrentPosition() > liftPosition) {
                    robot.winch.stopMotor();
                    telemetry.addData("UP: ", "Motors stopped");
                }
            } else if((liftPosition - robot.motorLiftFront.getCurrentPosition()) < 0){
                telemetry.addData("DOWN: Distance to target position = ", (liftPosition - robot.motorLiftFront.getCurrentPosition()));
                if (robot.motorLiftFront.getCurrentPosition() > liftPosition) {
                    liftPower = Math.abs(robot.winch.getCurrentPosition() - liftPosition) * l08_Kp;
                    robot.winch.set(-liftPower);
                    telemetry.addData("DOWN: Lift Power : ", liftPower);
                } else if (robot.motorLiftFront.getCurrentPosition() < liftPosition) {
                    robot.winch.stopMotor();
                    telemetry.addData("DOWN: ", "Motors stopped");
                }
            }

             */



            //robot.motorLiftRear.setTargetPosition(liftPosition);
            //robot.motorLiftRear.set(1);
            //robot.motorLiftFront.setVelocity(robot.motorLiftRear.getVelocity());


            /*
            if(robot.winch.atTargetPosition()){
                robot.winch.set(1);
            }else{
                robot.winch.stopMotor();
            }

             */

            // Provide user feedback
            telemetry.addData("A:", "Lift Reset");
            telemetry.addData("B:", "Lift Low");
            telemetry.addData("X:", "Lift Mid");
            telemetry.addData("Y:", "Lift High");
//            telemetry.addData("Lift Rear Encoder Value = ", robot.winch.getCurrentPosition());
            telemetry.addData("IMU Angle X = ", robot.imu.getAngles()[0]);
            telemetry.addData("IMU Angle Y = ", robot.imu.getAngles()[1]);
            telemetry.addData("IMU Angle Z = ", robot.imu.getAngles()[2]);
            telemetry.addData("Left Stick X = ", gp1.getLeftX());
            telemetry.addData("Left Stick Y = ", gp1.getLeftY());
            telemetry.addData("Right Stick X = ", gp1.getRightX());
            telemetry.addData("Right Stick Y = ", gp1.getRightY());
            telemetry.update();

            // post telemetry to FTC Dashboard as well
            dashTelemetry.put("01 - IMU Angle X = ", robot.imu.getAngles()[0]);
            dashTelemetry.put("02 - IMU Angle Y = ", robot.imu.getAngles()[1]);
            dashTelemetry.put("03 - IMU Angle Z = ", robot.imu.getAngles()[2]);
//            dashTelemetry.put("05 - Lift Rear Encoder Value = ", robot.motorLiftRear.getCurrentPosition());
            dashTelemetry.put("04 - Claw Value = ", robot.servoGrabber.getPosition());
            dashTelemetry.put("05 - GP1.Button.A = ", "RESET LIFT");
            dashTelemetry.put("06 - GP1.Button.B = ", "LIFT LOW JUNCTION");
            dashTelemetry.put("07 - GP1.Button.X = ", "LIFT MID JUNCTION");
            dashTelemetry.put("08 - GP1.Button.Y = ", "LIFT HIGH JUNCTION");
            dashTelemetry.put("09 - GP2.Button.A = ", "Custom Position - program stack cone levels");
            dashTelemetry.put("10 - Lift Power = ", liftPower);
            dashTelemetry.put("11 - motorLF encoder = ", robot.motorLF.getCurrentPosition());
            dashTelemetry.put("12 - motorLR encoder = ", robot.motorLR.getCurrentPosition());
            dashTelemetry.put("13 - motorRF encoder = ", robot.motorRF.getCurrentPosition());
            dashTelemetry.put("14 - motorRR encoder = ", robot.motorRR.getCurrentPosition());
            dashTelemetry.put("15 - target Lift position = ", liftPosition);
//            dashTelemetry.put("16 - Winch at Target Lift Position = ", robot.winch.atTargetPosition());
            dashTelemetry.put("16a - motorLiftF Position = ", motorLiftF.getCurrentPosition());
            dashTelemetry.put("16b - motorLiftF Get Velocity = ", motorLiftF.getVelocity());
            dashTelemetry.put("16c - motorLiftR Position = ", motorLiftR.getCurrentPosition());
            dashTelemetry.put("16d - motorLiftF Get Velocity = ", motorLiftR.getVelocity());
            dashboard.sendTelemetryPacket(dashTelemetry);

        }   // end of while(opModeIsActive)
        liftControl.stopThread();
    }   // end of runOpMode()

}       // end of BrokenBot class