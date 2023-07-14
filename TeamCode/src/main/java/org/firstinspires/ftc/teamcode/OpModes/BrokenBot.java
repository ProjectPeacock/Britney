package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.LiftControlClass;

import java.util.List;

@Config
@TeleOp(name = "Broken Bot", group = "Development")
@Disabled

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
    public static double l9_ARM_INTAKE=robot.SERVO_ARM_INTAKE;
    public static double l10_ARM_SCORE=robot.SERVO_ARM_SCORE;

    @Override
    public void runOpMode() {
        boolean fieldCentric = true;
        boolean liftToPosition = true;
        int liftPosition = 0;
        LinearOpMode opMode = this;

        dashboard = FtcDashboard.getInstance();
        TelemetryPacket dashTelemetry = new TelemetryPacket();

        robot.init(hardwareMap);
        robot.servoArm.setPosition(robot.SERVO_ARM_INTAKE);
        robot.servoGrabber.setPosition(robot.CLAW_OPEN);
        robot.servoFlipperLeft.setPosition(0);
        robot.servoFlipperRight.setPosition(1);
        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);
        ButtonReader clawToggleButton = new ButtonReader(gp1, GamepadKeys.Button.RIGHT_BUMPER);

        LiftControlClass drive = new LiftControlClass(robot);

        telemetry.addData("Ready to Run: ", "GOOD LUCK");
        telemetry.update();


        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        double liftPower=0;
        boolean clawToggle=false, clawReady=false, armToggle=false, armReady=false;

        waitForStart();

        while (opModeIsActive()) {


            telemetry.addData("Servo Arm = ", robot.servoArm.getPosition());

            if(clawToggleButton.isDown()&&clawReady){
                clawToggle=!clawToggle;
            }
            if(!clawToggleButton.isDown()){
                clawReady=true;
            }else{
                clawReady=false;
            }

            if (!clawToggle) {
                robot.servoGrabber.setPosition(l1_CLAW_OPEN);
            } else {
                robot.servoGrabber.setPosition(l2_CLAW_CLOSE);
            }


            if(gp1.getButton(GamepadKeys.Button.LEFT_BUMPER)&&armReady){
                armToggle=!armToggle;
            }
            if(!gp1.getButton(GamepadKeys.Button.LEFT_BUMPER)){
                armReady=true;
            }else{
                armReady=false;
            }

            if (!armToggle) {
                robot.servoArm.setPosition(l9_ARM_INTAKE);
            } else {
                robot.servoArm.setPosition(l10_ARM_SCORE);
            }

            // Provide user feedback
            telemetry.addData("A:", "Lift Reset");
            telemetry.addData("B:", "Lift Low");
            telemetry.addData("X:", "Lift Mid");
            telemetry.addData("Y:", "Lift High");

            telemetry.addData("IMU Angle X = ", robot.imu.getAngles()[0]);
            telemetry.addData("IMU Angle Y = ", robot.imu.getAngles()[1]);
            telemetry.addData("IMU Angle Z = ", robot.imu.getAngles()[2]);
            telemetry.addData("Left Stick X = ", gp1.getLeftX());
            telemetry.addData("Left Stick Y = ", gp1.getLeftY());
            telemetry.addData("Right Stick X = ", gp1.getRightX());
            telemetry.addData("Right Stick Y = ", gp1.getRightY());
            telemetry.addData("Lift Height: ",robot.motorLiftLeft.getCurrentPosition());
            telemetry.update();

            // post telemetry to FTC Dashboard as well
            dashTelemetry.put("01 - IMU Angle X = ", robot.imu.getAngles()[0]);
            dashTelemetry.put("02 - IMU Angle Y = ", robot.imu.getAngles()[1]);
            dashTelemetry.put("03 - IMU Angle Z = ", robot.imu.getAngles()[2]);

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
            dashTelemetry.put("19 - L/R Odometer = ",robot.motorRR.getCurrentPosition());
            dashTelemetry.put("20 - F/B Odometer = ",robot.motorLF.getCurrentPosition());
            dashboard.sendTelemetryPacket(dashTelemetry);

        }   // end of while(opModeIsActive)
    }   // end of runOpMode()

}       // end of BrokenBot class