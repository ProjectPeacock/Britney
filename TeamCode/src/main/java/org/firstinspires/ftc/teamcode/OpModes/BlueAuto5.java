package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Libs.AutoLiftControlClass;
import org.firstinspires.ftc.teamcode.Libs.AutoParams;
import org.firstinspires.ftc.teamcode.Libs.LiftControlClass;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.ArrayList;
@Autonomous(name = "Blue 1+5", group = "Competition")
public class BlueAuto5 extends OpMode{
    FtcDashboard dashboard;
    TelemetryPacket dashTelemetry = new TelemetryPacket();
    private String configFile="autoGyroValue.txt";

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int ID_TAG_OF_INTEREST = 1; // Tag ID 1 from the 36h11 family

    AprilTagDetection tagOfInterest = null;

    /*

    OPMODE MAP - PLEASE READ BEFORE EDITING

    This opMode uses TrajectorySequences from RoadRunner. They are made to be run back to back.
    The order of operations is: untilCycle -> firstCycle -> cycles2to4 (repeated 3 times) -> highCycle -> park

    Each parking position is its own TrajectorySequence. They are all made to be run following highCycle.

    All values for target position and heading come from AutoParams.java.

     */

    //lift control init
    public final static HWProfile robot = new HWProfile();
    private OpMode myOpmode=this;
    AutoLiftControlClass clawControl;

    //init params
    AutoParams params = new AutoParams();

    int parkPosition = 2;
    int liftTarget=0;

    TrajectorySequence trajectory1;
    TrajectorySequence trajectory2;
    TrajectorySequence trajectory3;
    TrajectorySequence trajectory4;
    TrajectorySequence trajectory5;
    TrajectorySequence trajectory6;

    SampleMecanumDrive drive;

    public void init(){
        robot.init(hardwareMap);
        clawControl = new AutoLiftControlClass(robot,myOpmode);
        robot.servoGrabber.setPosition(robot.CLAW_CLOSE);
        robot.servoArm.setPosition(robot.SERVO_ARM_INTAKE);
        dashboard = FtcDashboard.getInstance();
        TelemetryPacket dashTelemetry = new TelemetryPacket();

        drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose= new Pose2d(params.startPoseX,params.startPoseY,Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        trajectory1 = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0.45,()->{liftTarget=clawControl.moveLiftScore(2,robot.liftTicksPerInch*2,false);})

                .splineTo(new Vector2d(25.5,-26),Math.toRadians(120))
                .UNSTABLE_addTemporalMarkerOffset(0.25,()->{liftTarget=clawControl.moveLiftScore(2,robot.liftAdjust+robot.liftTicksPerInch*2,false);})
                .UNSTABLE_addTemporalMarkerOffset(0.45, clawControl::openClaw)
                .waitSeconds(0.5)
                /*
                .back(8)
                .UNSTABLE_addTemporalMarkerOffset(-0.125,()->{liftTarget=clawControl.moveLiftGrab();})
                .turn(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(55.5,-10,Math.toRadians(0)),Math.toRadians(0))
                .forward(6)

                .UNSTABLE_addTemporalMarkerOffset(0, clawControl::closeClaw)
                .waitSeconds(0.35)

                .back(30)
                .UNSTABLE_addTemporalMarkerOffset(0.25,()->{liftTarget=clawControl.moveLiftScore(3,true);})
                .splineToSplineHeading(new Pose2d(0,-24,Math.toRadians(45)),Math.toRadians(-120))
                .UNSTABLE_addTemporalMarkerOffset(0.25,()->{liftTarget=clawControl.moveLiftScore(3,robot.liftAdjust,true);})
                .UNSTABLE_addTemporalMarkerOffset(0.35, clawControl::openClaw)
                .waitSeconds(0.35)

                .UNSTABLE_addTemporalMarkerOffset(0.25,()->{liftTarget=clawControl.moveLiftGrab();})
                .splineToSplineHeading(new Pose2d(31.5,-10,Math.toRadians(0)),Math.toRadians(0))
                .forward(24)
                .waitSeconds(0.35)

                .back(30)
                .UNSTABLE_addTemporalMarkerOffset(0.25,()->{liftTarget=clawControl.moveLiftScore(3,true);})
                .splineToSplineHeading(new Pose2d(0,-24,Math.toRadians(45)),Math.toRadians(-120))
                .UNSTABLE_addTemporalMarkerOffset(0.25,()->{liftTarget=clawControl.moveLiftScore(3,robot.liftAdjust,true);})
                .UNSTABLE_addTemporalMarkerOffset(0.35, clawControl::openClaw)
                .waitSeconds(0.35)

                .UNSTABLE_addTemporalMarkerOffset(0.25,()->{liftTarget=clawControl.moveLiftGrab();})
                .splineToSplineHeading(new Pose2d(31.5,-10,Math.toRadians(0)),Math.toRadians(0))
                .forward(24)
                .waitSeconds(0.35)

                .back(30)
                .UNSTABLE_addTemporalMarkerOffset(0.25,()->{liftTarget=clawControl.moveLiftScore(3,true);})
                .splineToSplineHeading(new Pose2d(0,-24,Math.toRadians(45)),Math.toRadians(-120))
                .UNSTABLE_addTemporalMarkerOffset(0.25,()->{liftTarget=clawControl.moveLiftScore(3,robot.liftAdjust,true);})
                .UNSTABLE_addTemporalMarkerOffset(0.35, clawControl::openClaw)
                .waitSeconds(0.35)

                .UNSTABLE_addTemporalMarkerOffset(0.25,()->{liftTarget=clawControl.moveLiftGrab();})
                .splineToSplineHeading(new Pose2d(31.5,-10,Math.toRadians(0)),Math.toRadians(0))
                .forward(24)
                .waitSeconds(0.35)

                .back(30)
                .UNSTABLE_addTemporalMarkerOffset(0.25,()->{liftTarget=clawControl.moveLiftScore(3,true);})
                .splineToSplineHeading(new Pose2d(0,-24,Math.toRadians(45)),Math.toRadians(-120))
                .UNSTABLE_addTemporalMarkerOffset(0.25,()->{liftTarget=clawControl.moveLiftScore(3,robot.liftAdjust,true);})
                .UNSTABLE_addTemporalMarkerOffset(0.35, clawControl::openClaw)
                .waitSeconds(0.35)

                .UNSTABLE_addTemporalMarkerOffset(0.25,()->{liftTarget=clawControl.moveLiftGrab();})
                .splineToSplineHeading(new Pose2d(31.5,-10,Math.toRadians(0)),Math.toRadians(0))
                .forward(24)
                .waitSeconds(0.35)

                .back(30)
                .UNSTABLE_addTemporalMarkerOffset(0.25,()->{liftTarget=clawControl.moveLiftScore(3,true);})
                .splineToSplineHeading(new Pose2d(0,-24,Math.toRadians(45)),Math.toRadians(-120))
                .UNSTABLE_addTemporalMarkerOffset(0.25,()->{liftTarget=clawControl.moveLiftScore(3,robot.liftAdjust,true);})
                .UNSTABLE_addTemporalMarkerOffset(0.35, clawControl::openClaw)
                .waitSeconds(0.35)
                .UNSTABLE_addTemporalMarkerOffset(0.35, clawControl::closeClaw)
                .UNSTABLE_addTemporalMarkerOffset(0.25,()->{liftTarget=clawControl.moveLiftScore(0,false);})
                //.splineToSplineHeading(new Pose2d(12,-12,Math.toRadians(90)),Math.toRadians(45))
                */
                .build();

    drive.followTrajectorySequenceAsync(trajectory1);
    telemetry.addData("Ready to ","run");
    telemetry.update();
    }
    public void loop(){
        drive.update();
        clawControl.runTo(liftTarget);
    }

}
