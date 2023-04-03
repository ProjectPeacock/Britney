package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
@Config
@TeleOp(name = "Lift Test", group = "Competition")
//@Disabled
public class LiftTest extends LinearOpMode {
    public DcMotorEx motorLiftLeft = null;
    public DcMotorEx motorLiftRight = null;
    public MotorGroup winch = null;
    FtcDashboard dashboard;
    //public MotorGroup winch = null;
    public static int LIFT_BOTTOM=0;
    public static int LIFT_LOW=100;
    public static int LIFT_MID=500;
    public static int LIFT_HIGH=800;
    public static double LIFT_POW_DOWN=0.3;
    public static double LIFT_POW_UP=1;
    public static double LIFT_KP=0.05;
    public static int LIFT_TOL=10;


    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        TelemetryPacket dashTelemetry = new TelemetryPacket();

        double liftPower=0;
        int liftPos=0;
        int lastPosition=0;

        //lift motor init
        motorLiftLeft = hardwareMap.get(DcMotorEx.class, "motorLiftLeft");
        motorLiftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLiftRight = hardwareMap.get(DcMotorEx.class, "motorLiftRight");
        motorLiftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        telemetry.addData("Ready to Run: ", "GOOD LUCK");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.left_trigger>0.1){
                motorLiftLeft.setPower(-gamepad1.left_trigger);
                motorLiftRight.setPower(-gamepad1.left_trigger);
            }else if(gamepad1.right_trigger>0.1){
                motorLiftLeft.setPower(gamepad1.right_trigger);
                motorLiftRight.setPower(gamepad1.right_trigger);
            }else{
                motorLiftLeft.setPower(0);
                motorLiftRight.setPower(0);
            }

            telemetry.addData("Left motor posiition: ",motorLiftLeft.getCurrentPosition());
            telemetry.addData("Right motor position: ",motorLiftRight.getCurrentPosition());
            telemetry.update();
        }   // end of while(opModeIsActive)
    }   // end of runOpMode()
}       // end of LiftTest class