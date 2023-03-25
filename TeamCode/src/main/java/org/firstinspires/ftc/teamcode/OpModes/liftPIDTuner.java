package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class liftPIDTuner extends OpMode {
    private PIDController controller;
    public static double p=0,i=0,d=0,f=0.005;
    public static int target=0;

    private final double ticksInDegrees=8192/360;

    private DcMotorEx motorLiftLeft;
    private DcMotorEx motorLiftRight;

    @Override
    public void init(){
        controller=new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motorLiftLeft=hardwareMap.get(DcMotorEx.class,"motorLiftLeft");
        motorLiftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLiftRight=hardwareMap.get(DcMotorEx.class,"motorLiftRight");
        motorLiftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLiftRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    @Override
    public void loop(){
        controller.setPID(p,i,d);
        int armPos= motorLiftLeft.getCurrentPosition();
        double pid=controller.calculate(armPos,target);
        double ff = Math.cos(Math.toRadians(target/ticksInDegrees))*f;

        double power=pid+ff;
        motorLiftLeft.setPower(power);
        motorLiftRight.setPower(power);

        telemetry.addData("pos: ",armPos);
        telemetry.addData("target: ",target);
        telemetry.update();
    }
}
