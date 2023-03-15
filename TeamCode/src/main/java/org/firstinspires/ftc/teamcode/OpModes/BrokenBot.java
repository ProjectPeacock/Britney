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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.LiftControlClass;

import java.util.List;

@Config
@TeleOp(name = "Broken Bot", group = "Development")

public class BrokenBot extends LinearOpMode {
    private final static HWProfile robot = new HWProfile();
    FtcDashboard dashboard;
    public static double l1_CLAW_OPEN = robot.CLAW_OPEN;
    public static double l2_CLAW_CLOSE = robot.CLAW_CLOSE;
    public static int l3_LIFT_JUNCTION_HIGH = robot.LIFT_HIGH;
    public static int l4_LIFT_JUNCTION_MID = robot.LIFT_MID;
    public static int l5_LIFT_JUNCTION_LOW = robot.LIFT_LOW;
    public static int l6_LIFT_BOTTOM = robot.LIFT_BOTTOM;
    public static double l7_LIFT_POWER = robot.LIFT_POW;
    public static double l8_TURN_MULTIPLIER = robot.TURN_MULTIPLIER;

    @Override
    public void runOpMode() {
        boolean fieldCentric = true;
        boolean liftToPosition = true;
        int liftPosition = 0;
        LinearOpMode opMode = this;

        dashboard = FtcDashboard.getInstance();
        TelemetryPacket dashTelemetry = new TelemetryPacket();

        robot.init(hardwareMap);
        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);
        ButtonReader clawToggleButton = new ButtonReader(gp1, GamepadKeys.Button.RIGHT_BUMPER);

        LiftControlClass drive = new LiftControlClass(robot, opMode);

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

        waitForStart();

        while (opModeIsActive()) {


            telemetry.addData("Servo Align = ", robot.servoAlign.getPosition());
            if (gamepad2.left_bumper) {
                robot.servoAlign.setPosition(robot.SERVO_ALIGN_DOWN);
            } else if (gamepad2.right_bumper){
                robot.servoAlign.setPosition(robot.SERVO_ALIGN_UP);
            }

            if (gamepad2.right_trigger>0.1) {
                robot.autoLight.set(-1);
            }else{
                robot.autoLight.set(0);
            }
            if(fieldCentric){
                robot.mecanum.driveFieldCentric(gp1.getLeftX(),gp1.getLeftY(),-gp1.getRightX()*l8_TURN_MULTIPLIER,robot.imu.getRotation2d().getDegrees()+180, true);
            }else{
                robot.mecanum.driveRobotCentric(gp1.getLeftX(),gp1.getLeftY(),-gp1.getRightX()*l8_TURN_MULTIPLIER, true);
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
                robot.servoGrabber.setPosition(l1_CLAW_OPEN);
            } else {
                robot.servoGrabber.setPosition(l2_CLAW_CLOSE);
            }

            if (gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1) {
                liftPosition = liftPosition - 10;
                liftPower = l7_LIFT_POWER;

            } else if (gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1) {
                liftPosition = liftPosition + 10;
                liftPower = l7_LIFT_POWER;
            } else {
            }

            if(gp1.isDown(GamepadKeys.Button.A)){
                liftPosition = robot.LIFT_BOTTOM;
                liftPower = l7_LIFT_POWER;
            }


            if(gp1.isDown(GamepadKeys.Button.Y)){
                liftPosition =l3_LIFT_JUNCTION_HIGH;
                liftPower = l7_LIFT_POWER;
            }

            if(gp2.isDown(GamepadKeys.Button.A)){
                liftPosition = l6_LIFT_BOTTOM;
                liftPower = l7_LIFT_POWER;
            }

            liftPosition = Range.clip(liftPosition, robot.LIFT_BOTTOM, robot.MAX_LIFT_VALUE);

            robot.motorLiftFront.setTargetPosition(liftPosition);
            robot.motorLiftRear.setTargetPosition(liftPosition);
            robot.motorLiftFront.setPower(liftPower);
            robot.motorLiftRear.setPower(liftPower);

            // Provide user feedback
            telemetry.addData("A:", "Lift Reset");
            telemetry.addData("B:", "Lift Low");
            telemetry.addData("X:", "Lift Mid");
            telemetry.addData("Y:", "Lift High");
            telemetry.addData("Lift Front Encoder Value = ", robot.motorLiftFront.getCurrentPosition());
            telemetry.addData("Lift Rear Encoder Value = ", robot.motorLiftRear.getCurrentPosition());
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
            dashTelemetry.put("04 - Lift Front Encoder Value = ", robot.motorLiftFront.getCurrentPosition());
            dashTelemetry.put("05 - Lift Rear Encoder Value = ", robot.motorLiftRear.getCurrentPosition());
            dashTelemetry.put("06 - Claw Value = ", robot.servoGrabber.getPosition());
            dashTelemetry.put("07 - GP1.Button.A = ", "RESET LIFT");
            dashTelemetry.put("08 - GP1.Button.B = ", "LIFT LOW JUNCTION");
            dashTelemetry.put("09 - GP1.Button.X = ", "LIFT MID JUNCTION");
            dashTelemetry.put("10 - GP1.Button.Y = ", "LIFT HIGH JUNCTION");
            dashTelemetry.put("11 - GP2.Button.A = ", "Custom Position - program stack cone levels");
            dashTelemetry.put("12 - Lift Power = ", liftPower);
            dashTelemetry.put("13 - motorLF encoder = ", robot.motorLF.getCurrentPosition());
            dashTelemetry.put("14 - motorLR encoder = ", robot.motorLR.getCurrentPosition());
            dashTelemetry.put("15 - motorRF encoder = ", robot.motorRF.getCurrentPosition());
            dashTelemetry.put("16 - motorRR encoder = ", robot.motorRR.getCurrentPosition());
            dashTelemetry.put("17 - motorLiftFront position = ", robot.motorLiftFront.getCurrentPosition());
            dashTelemetry.put("18 - motorLiftRear position = ", robot.motorLiftFront.getCurrentPosition());
            dashTelemetry.put("19 - L/R Odometer = ",robot.motorRR.getCurrentPosition());
            dashTelemetry.put("20 - F/B Odometer = ",robot.motorLF.getCurrentPosition());
            dashboard.sendTelemetryPacket(dashTelemetry);

        }   // end of while(opModeIsActive)
    }   // end of runOpMode()

}       // end of BrokenBot class