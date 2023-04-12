package org.firstinspires.ftc.teamcode.Hardware;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
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

    //servo flipper positions
    public final double SERVO_FLIPPER_DOWN=0.4;
    public final double SERVO_FLIPPER_UP=0;

    // servo arm positions
    public final double SERVO_ARM_INTAKE = 0.92;
    public final double SERVO_ARM_SCORE = 0.0;
    public final double SERVO_ARM_DUNK = 0.0;

    //tflite file name
    public final String tfliteFileName = "PP_Generic_SS.tflite";

    //max difference between front and rear lift motors
    public final int rearLiftMotorTol = 8;

    //claw positions
    public final double CLAW_OPEN =0.55;
    public final double CLAW_CLOSE =0.2;
    public final double CLAW_BEACON=0.4;


    //drive constants
    public final double TURN_MULTIPLIER = 0.75;

    //anti-tip constants
    public final double ANTI_TIP_ADJ=0.3;
    public final double ANTI_TIP_TOL=10;
    public final int ANTI_TIP_AXIS=1;

    //lift constants
    final public int liftAdjust=750;
    final public double LIFT_POW=1;
    final public int MAX_LIFT_VALUE = -57500;
    public final int ARM_THRESHOLD =-15600;
    final public int LIFT_BOTTOM=-750;
    final public int LIFT_LOW=-20350;
    final public int LIFT_MID=-38300;
    final public int LIFT_HIGH=-56000;

    //final private int liftTicksPerInch= (int) 8192/4.3267;
    final public int liftTicksPerInch= 1893;
    final public int stack1=(int)-5.25*liftTicksPerInch;
    final public int stack2=(int)-3.75*liftTicksPerInch;
    final public int stack3=(int)-3*liftTicksPerInch;
    final public int stack4=(int)-0.75*liftTicksPerInch;

    public RevColorSensorV3 sensorColor;
    //public OpticalDistanceSensor sensorDistance;

    /* Public OpMode members. */
    public MotorEx motorLF = null;
    public MotorEx motorLR = null;
    public MotorEx motorRF = null;
    public MotorEx motorRR = null;
    private MotorEx motorLiftLeft = null;
    private MotorEx motorLiftRight = null;
    public MotorGroup lift = null;
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
        motorLiftLeft = new MotorEx(ahwMap, "motorLiftLeft", 8192,6000);
        motorLiftRight = new MotorEx(ahwMap, "motorLiftRight", 8192,6000);

        lift = new MotorGroup(motorLiftLeft,motorLiftRight);
        lift.setRunMode(Motor.RunMode.PositionControl);
        lift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);;
        lift.resetEncoder();
        lift.setPositionTolerance(500);
        lift.setPositionCoefficient(0.025);


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