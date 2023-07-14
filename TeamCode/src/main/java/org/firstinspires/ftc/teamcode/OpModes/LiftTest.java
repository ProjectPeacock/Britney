package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
@Disabled
public class LiftTest extends OpMode {
    private PIDController controller;
    public static double kP=0;
    public static int tolerance=0;
    public static double p=0.002, i=0, d=0;
    public static double f=-0.2;

    public static int target=0;

    private final double ticks_in_degrees = 8192/360;

    private MotorEx motorLiftLeft;
    private MotorEx motorLiftRight;
    private MotorGroup lift;
    @Override
    public void init(){
        controller=new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motorLiftLeft= new MotorEx(hardwareMap, "motorLiftLeft", 8192,6000);
        motorLiftRight= new MotorEx(hardwareMap, "motorLiftRight", 8192,6000);

        lift = new MotorGroup(motorLiftLeft,motorLiftRight);
        //lift.setRunMode(Motor.RunMode.PositionControl);
        //lift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);;
        lift.resetEncoder();
        //lift.setPositionTolerance(tolerance);
        //lift.setPositionCoefficient(kP);
    }

    @Override
    public void loop(){
        //lift.setPositionTolerance(tolerance);
        //lift.setPositionCoefficient(kP);
        int armPos = motorLiftLeft.getCurrentPosition();

        controller.setPID(p,i,d);
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians((target/ticks_in_degrees)))*f;

        lift.set(pid+ff);

        /*
        lift.setTargetPosition(target);
        if(!lift.atTargetPosition()){
            lift.set(1);
        }else{
            lift.set(0);
        }

         */

        telemetry.addData("pos: ",armPos);
        telemetry.addData("target: ",target);
    }

}
