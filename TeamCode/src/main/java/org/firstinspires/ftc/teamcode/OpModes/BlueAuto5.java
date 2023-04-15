package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
@Autonomous(name = "Blue 1+5", group = "Competition")
public class BlueAuto5 extends OpMode {
    FtcDashboard dashboard;
    TelemetryPacket dashTelemetry = new TelemetryPacket();
    private String configFile = "autoGyroValue.txt";

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
    private OpMode myOpmode = this;
    AutoLiftControlClass clawControl;

    //init params
    AutoParams params = new AutoParams();

    int parkPosition = 2;
    int liftTarget = 0;

    TrajectorySequence trajectory1;
    TrajectorySequence trajectory2;
    TrajectorySequence trajectory3;
    TrajectorySequence trajectory4;
    TrajectorySequence trajectory5;
    TrajectorySequence trajectory6;

    SampleMecanumDrive drive;

    public void init() {
        robot.init(hardwareMap);
        robot.autoLight.set(-1);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        clawControl = new AutoLiftControlClass(robot, myOpmode);
        robot.servoGrabber.setPosition(robot.CLAW_CLOSE);
        robot.servoArm.setPosition(robot.SERVO_ARM_INTAKE);
        clawControl.flippersUp();
        dashboard = FtcDashboard.getInstance();
        TelemetryPacket dashTelemetry = new TelemetryPacket();

        drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(params.startPoseX, params.startPoseY, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        trajectory1 = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(27.5, -27), Math.toRadians(140))
                .UNSTABLE_addTemporalMarkerOffset(-.25, () -> {
                    liftTarget = clawControl.moveLiftScore(2, robot.liftTicksPerInch * 4, false);
                })
                //.UNSTABLE_addTemporalMarkerOffset(0.125,()->{liftTarget=clawControl.moveLiftScore(2,robot.liftTicksPerInch*3-7500,false);})

                .UNSTABLE_addTemporalMarkerOffset(0.65, clawControl::openClaw)
                .waitSeconds(0.75)

                .back(8)
                .UNSTABLE_addTemporalMarkerOffset(-0.25, () -> {
                    liftTarget = clawControl.moveLiftGrab();
                })
                .turn(Math.toRadians(-30))
                .splineToLinearHeading(new Pose2d(36, -12, Math.toRadians(90)), Math.toRadians(90))
                .strafeLeft(24)
                .build();

        trajectory2 = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(27.5, -27), Math.toRadians(140))
                .UNSTABLE_addTemporalMarkerOffset(-.25, () -> {
                    liftTarget = clawControl.moveLiftScore(2, robot.liftTicksPerInch * 4, false);
                })
                //.UNSTABLE_addTemporalMarkerOffset(0.125,()->{liftTarget=clawControl.moveLiftScore(2,robot.liftTicksPerInch*3-7500,false);})

                .UNSTABLE_addTemporalMarkerOffset(0.65, clawControl::openClaw)
                .waitSeconds(0.75)

                .back(8)
                .UNSTABLE_addTemporalMarkerOffset(-0.25, () -> {
                    liftTarget = clawControl.moveLiftGrab();
                })
                .turn(Math.toRadians(-30))
                .splineToLinearHeading(new Pose2d(36, -12, Math.toRadians(90)), Math.toRadians(90))
                .build();

        trajectory3 = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(27.5, -27), Math.toRadians(140))
                .UNSTABLE_addTemporalMarkerOffset(-.25, () -> {
                    liftTarget = clawControl.moveLiftScore(2, robot.liftTicksPerInch * 4, false);
                })
                //.UNSTABLE_addTemporalMarkerOffset(0.125,()->{liftTarget=clawControl.moveLiftScore(2,robot.liftTicksPerInch*3-7500,false);})

                .UNSTABLE_addTemporalMarkerOffset(0.65, clawControl::openClaw)
                .waitSeconds(0.75)

                .back(8)
                .UNSTABLE_addTemporalMarkerOffset(-0.25, () -> {
                    liftTarget = clawControl.moveLiftGrab();
                })
                .turn(Math.toRadians(-30))
                .splineToLinearHeading(new Pose2d(36, -12, Math.toRadians(90)), Math.toRadians(90))
                .strafeRight(24)

                /*
                //CYCLE 1
                .splineToLinearHeading(new Pose2d(69,-10,Math.toRadians(0)),Math.toRadians(0))

                .UNSTABLE_addTemporalMarkerOffset(0, clawControl::closeClaw)
                .waitSeconds(0.35)

                .UNSTABLE_addTemporalMarkerOffset(0.25,()->{liftTarget=clawControl.moveLiftScore(2,false);})
                .waitSeconds(0.35)

                .back(6)
                .UNSTABLE_addTemporalMarkerOffset(0.25,()->{liftTarget=clawControl.moveLiftScore(2,true);})
                .splineToSplineHeading(new Pose2d(35.5,-13,Math.toRadians(10)),Math.toRadians(-120))

                .UNSTABLE_addTemporalMarkerOffset(0.25,()->{liftTarget=clawControl.moveLiftScore(2,-7500,true);})
                .UNSTABLE_addTemporalMarkerOffset(0.35, clawControl::openClaw)
                .waitSeconds(0.35)
                .splineToLinearHeading(new Pose2d(69,-10,Math.toRadians(0)),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0.25,()->{liftTarget=clawControl.moveLiftGrab();})
/*
                //CYCLE 2
                .splineToLinearHeading(new Pose2d(75.5,-10,Math.toRadians(0)),Math.toRadians(0))

                .UNSTABLE_addTemporalMarkerOffset(0, clawControl::closeClaw)
                .waitSeconds(0.5)

                .UNSTABLE_addTemporalMarkerOffset(0.25,()->{liftTarget=clawControl.moveLiftScore(2,false);})
                .waitSeconds(0.5)


                .UNSTABLE_addTemporalMarkerOffset(0.25,()->{liftTarget=clawControl.moveLiftScore(2,true);})
                .splineToSplineHeading(new Pose2d(43.5,-18,Math.toRadians(45)),Math.toRadians(-120))

                .UNSTABLE_addTemporalMarkerOffset(0.25,()->{liftTarget=clawControl.moveLiftScore(2,-7500,true);})
                .UNSTABLE_addTemporalMarkerOffset(0.35, clawControl::openClaw)
                .waitSeconds(0.35)
                .UNSTABLE_addTemporalMarkerOffset(0.25,()->{liftTarget=clawControl.moveLiftGrab();})
                .splineToSplineHeading(new Pose2d(69.5,-8.5,Math.toRadians(0)),Math.toRadians(0))
*/
                .build();
    }

    public void init_loop() {

        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if (currentDetections.size() != 0) {
            boolean tagFound = false;

            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == 1 || tag.id == 2 || tag.id == 3) {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }

            {
                if (tagFound) {
                    switch (tagOfInterest.id) {
                        case 1:
                            parkPosition = 1;
                            break;
                        case 2:
                            parkPosition = 2;
                            break;
                        case 3:
                            parkPosition = 3;
                            break;
                    }
                    telemetry.addLine("Tag of interest is in sight!\n\n:");
                    telemetry.addData("tag ID=", tagOfInterest.id);
                    telemetry.addData("park position= ", parkPosition);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");
                }

            }
        } else {
            telemetry.addLine("Don't see any tags of interest :(");
        }
        telemetry.update();

    }
    public void start(){
        if (parkPosition == 1) {
            drive.followTrajectorySequenceAsync(trajectory1);
        } else if (parkPosition == 3) {
            drive.followTrajectorySequenceAsync(trajectory3);
        } else {
            drive.followTrajectorySequenceAsync(trajectory2);
        }
    }

    public void loop(){
        drive.update();
        clawControl.runTo(liftTarget);
    }

}
