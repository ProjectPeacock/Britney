package org.firstinspires.ftc.teamcode.Hardware;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class HWProfile {
    //constants
    public final boolean fieldCentric=true;

    //lift PIDF coefficients
    public final double kP=0;
    public final double kI=0;
    public final double kD=0;
    public final double kF=0;

    public final double liftTicks=8192/360.0;

    //aligner constants
    public final int ARM_THRESHOLD =208;
    // servo align positions
    public final double SERVO_ARM_INTAKE = 1;
    public final double SERVO_ARM_SCORE = 0.05;
    public final double SERVO_ARM_DUNK = 0.05;

    //tflite file name
    public final String tfliteFileName = "PP_Generic_SS.tflite";

    //max difference between front and rear lift motors
    public final int rearLiftMotorTol = 8;

    //claw positions
    public final double CLAW_OPEN =0.65;
    public final double CLAW_CLOSE =0.3;
    public final double CLAW_BEACON=0.4;


    //odometer positions
    public final double ODO_UP=0.25;
    public final double ODO_DOWN=0.95;

    //drive constants
    public final double TURN_MULTIPLIER = 0.75;

    //anti-tip constants
    public final double ANTI_TIP_ADJ=0.3;
    public final double ANTI_TIP_TOL=10;
    public final int ANTI_TIP_AXIS=1;

    //lift constants
    final public int liftAdjust=15;
    final public double LIFT_POW=1;
    final public int MAX_LIFT_VALUE = 715;
    final public int LIFT_BOTTOM=0;
    final public int LIFT_LOW=247;
    final public int LIFT_MID=475;
    final public int LIFT_HIGH=700;

    final private int liftTicksPerInch=38;
    final public int stack1=(int)5.25*liftTicksPerInch;
    final public int stack2=(int)3.75*liftTicksPerInch;
    final public int stack3=(int)3*liftTicksPerInch;
    final public int stack4=(int)0.75*liftTicksPerInch;

    public RevColorSensorV3 sensorColor;
    //public OpticalDistanceSensor sensorDistance;

    /* Public OpMode members. */
    public MotorEx motorLF = null;
    public MotorEx motorLR = null;
    public MotorEx motorRF = null;
    public MotorEx motorRR = null;
    public DcMotorEx motorLiftLeft = null;
    public DcMotorEx motorLiftRight = null;
    public RevIMU imu = null;
    public ServoImplEx servoGrabber = null;
    public ServoImplEx servoArm = null;
    public ServoImplEx servoFlipperLeft;
    public ServoImplEx servoFlipperRight;
    public MecanumDrive mecanum = null;
    public MotorEx autoLight = null;
    public RevBlinkinLedDriver blinkin = null;
    public RevBlinkinLedDriver.BlinkinPattern pattern = null;

    /* local OpMode members. */
    HardwareMap hwMap =  null;

    /* Constructor */
    public HWProfile(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        //drive motor init
        motorLF = new MotorEx(ahwMap, "motorLF", Motor.GoBILDA.RPM_1150);
        motorLF.setRunMode(Motor.RunMode.RawPower);
        motorLF.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorLF.resetEncoder();

        motorLR = new MotorEx(ahwMap, "motorLR", Motor.GoBILDA.RPM_1150);
        motorLR.setRunMode(Motor.RunMode.RawPower);
        motorLR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorLR.resetEncoder();

        motorRF = new MotorEx(ahwMap, "motorRF", Motor.GoBILDA.RPM_1150);
        motorRF.setRunMode(Motor.RunMode.RawPower);
        motorRF.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorRF.resetEncoder();

        motorRR = new MotorEx(ahwMap, "motorRR", Motor.GoBILDA.RPM_1150);
        motorRR.setRunMode(Motor.RunMode.RawPower);
        motorRR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorRR.resetEncoder();

        //drivebase init
        mecanum = new MecanumDrive(motorLF, motorRF, motorLR, motorRR);

        //lift motors init
        motorLiftLeft = hwMap.get(DcMotorEx.class, "motorLiftLeft");
        motorLiftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftLeft.setPower(0);
        motorLiftLeft.setTargetPosition(0);
        motorLiftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLiftRight = hwMap.get(DcMotorEx.class, "motorLiftRight");
        motorLiftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftRight.setPower(0);
        motorLiftRight.setTargetPosition(0);
        motorLiftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLiftRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // rev color sensor
        sensorColor = hwMap.get(RevColorSensorV3.class, "sensorColor");

        //light init
        autoLight = new MotorEx(ahwMap, "sideSideOdo", 8192,10000);
        autoLight.set(0);

        //init servos
        servoGrabber = hwMap.get(ServoImplEx.class, "servoGrabber");
        servoArm = hwMap.get(ServoImplEx.class, "servoArm");
        servoFlipperLeft = hwMap.get(ServoImplEx.class, "servoFlipperLeft");
        servoFlipperRight = hwMap.get(ServoImplEx.class, "servoFlipperRight");

        //init blinkin

        blinkin = hwMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        blinkin.setPattern(pattern);

        //init imu
        imu = new RevIMU(ahwMap);
        imu.init();
    }
}  // end of HWProfile Class