package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Libs.AutoLiftControlClass;
import org.firstinspires.ftc.teamcode.Libs.AutoParams;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Red 1+3", group = "Competition")
public class TurnRedAuto extends OpMode {
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
        //robot.servoGrabber.setPosition(robot.CLAW_CLOSE);
        robot.servoArm.setPosition(robot.SERVO_ARM_INTAKE);
        clawControl.flippersUp();
        dashboard = FtcDashboard.getInstance();
        TelemetryPacket dashTelemetry = new TelemetryPacket();

        drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-params.startPoseX, params.startPoseY, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        trajectory1 = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0, clawControl::closeClaw)
                //.turn(Math.toRadians(90))
                .lineToSplineHeading(new Pose2d(-36,-40,Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-44,-0,Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(-.25, () -> {
                    liftTarget = clawControl.moveLiftScore(2,(int)(robot.liftTicksPerInch*0), true);
                })
                .lineToSplineHeading(new Pose2d(-33.75,-15.75,Math.toRadians(135)))

                .UNSTABLE_addTemporalMarkerOffset(0.0, clawControl::openClaw)
                .waitSeconds(0.15)

                //CYCLE 1
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {liftTarget = clawControl.moveLiftGrab();})
                .splineToLinearHeading(new Pose2d(-70.0,-7,Math.toRadians(180)),Math.toRadians(170))

                .UNSTABLE_addTemporalMarkerOffset(0.25, clawControl::closeClaw)

                .UNSTABLE_addTemporalMarkerOffset(.45, () -> {liftTarget = clawControl.moveLiftScore(1,false);})
                .waitSeconds(0.6)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {liftTarget = clawControl.moveLiftScore(2,true);})
                .back(6)
                .splineTo(new Vector2d(-33,-15),Math.toRadians(-45))
                .UNSTABLE_addTemporalMarkerOffset(0.0, clawControl::openClaw)
                .waitSeconds(0.15)

                //CYCLE 2
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {liftTarget = clawControl.moveLiftGrab();})
                .splineToLinearHeading(new Pose2d(-71.5,-8.5,Math.toRadians(180)),Math.toRadians(170))

                .UNSTABLE_addTemporalMarkerOffset(0.25, clawControl::closeClaw)

                .UNSTABLE_addTemporalMarkerOffset(.45, () -> {liftTarget = clawControl.moveLiftScore(1,false);})
                .waitSeconds(0.6)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {liftTarget = clawControl.moveLiftScore(2,true);})
                .back(6)
                .splineTo(new Vector2d(-34,-16.5),Math.toRadians(-45))
                .UNSTABLE_addTemporalMarkerOffset(0.0, clawControl::openClaw)
                .waitSeconds(0.15)

                //CYCLE 3
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {liftTarget = clawControl.moveLiftGrab();})
                .splineToLinearHeading(new Pose2d(-72.5,-10.5,Math.toRadians(180)),Math.toRadians(170))

                .UNSTABLE_addTemporalMarkerOffset(0.25, clawControl::closeClaw)

                .UNSTABLE_addTemporalMarkerOffset(.45, () -> {liftTarget = clawControl.moveLiftScore(1,false);})
                .waitSeconds(0.6)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {liftTarget = clawControl.moveLiftScore(2,true);})
                .back(6)
                .splineTo(new Vector2d(-34,-19),Math.toRadians(-45))
                .UNSTABLE_addTemporalMarkerOffset(0.0, clawControl::openClaw)
                .waitSeconds(0.15)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {liftTarget = clawControl.moveLiftScore(0,false);})

//                .splineTo(new Vector2d(-44,-12),Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-44,-12,Math.toRadians(90)),Math.toRadians(135))
// park
                .lineToSplineHeading(new Pose2d(-77,-13,Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {stop();})
                .build();

        trajectory2 = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0, clawControl::closeClaw)
                //.turn(Math.toRadians(90))
                .lineToSplineHeading(new Pose2d(-36,-40,Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-44,-0,Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(-.25, () -> {
                    liftTarget = clawControl.moveLiftScore(2,(int)(robot.liftTicksPerInch*0), true);
                })
                .lineToSplineHeading(new Pose2d(-33.75,-15.75,Math.toRadians(135)))

                .UNSTABLE_addTemporalMarkerOffset(0.0, clawControl::openClaw)
                .waitSeconds(0.15)

                //CYCLE 1
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {liftTarget = clawControl.moveLiftGrab();})
                .splineToLinearHeading(new Pose2d(-70.0,-7,Math.toRadians(180)),Math.toRadians(170))

                .UNSTABLE_addTemporalMarkerOffset(0.25, clawControl::closeClaw)

                .UNSTABLE_addTemporalMarkerOffset(.45, () -> {liftTarget = clawControl.moveLiftScore(1,false);})
                .waitSeconds(0.6)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {liftTarget = clawControl.moveLiftScore(2,true);})
                .back(6)
                .splineTo(new Vector2d(-33,-15),Math.toRadians(-45))
                .UNSTABLE_addTemporalMarkerOffset(0.0, clawControl::openClaw)
                .waitSeconds(0.15)

                //CYCLE 2
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {liftTarget = clawControl.moveLiftGrab();})
                .splineToLinearHeading(new Pose2d(-71.5,-8.5,Math.toRadians(180)),Math.toRadians(170))

                .UNSTABLE_addTemporalMarkerOffset(0.25, clawControl::closeClaw)

                .UNSTABLE_addTemporalMarkerOffset(.45, () -> {liftTarget = clawControl.moveLiftScore(1,false);})
                .waitSeconds(0.6)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {liftTarget = clawControl.moveLiftScore(2,true);})
                .back(6)
                .splineTo(new Vector2d(-34,-16.5),Math.toRadians(-45))
                .UNSTABLE_addTemporalMarkerOffset(0.0, clawControl::openClaw)
                .waitSeconds(0.15)

                //CYCLE 3
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {liftTarget = clawControl.moveLiftGrab();})
                .splineToLinearHeading(new Pose2d(-72.5,-10.5,Math.toRadians(180)),Math.toRadians(170))

                .UNSTABLE_addTemporalMarkerOffset(0.25, clawControl::closeClaw)

                .UNSTABLE_addTemporalMarkerOffset(.45, () -> {liftTarget = clawControl.moveLiftScore(1,false);})
                .waitSeconds(0.6)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {liftTarget = clawControl.moveLiftScore(2,true);})
                .back(6)
                .splineTo(new Vector2d(-34,-19),Math.toRadians(-45))
                .UNSTABLE_addTemporalMarkerOffset(0.0, clawControl::openClaw)
                .waitSeconds(0.15)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {liftTarget = clawControl.moveLiftScore(0,false);})

//                .splineTo(new Vector2d(-44,-12),Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-46,-12,Math.toRadians(90)),Math.toRadians(135))
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {stop();})
                .build();

        trajectory3 = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0, clawControl::closeClaw)
                //.turn(Math.toRadians(90))
                .lineToSplineHeading(new Pose2d(-36,-40,Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-44,-0,Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(-.25, () -> {
                    liftTarget = clawControl.moveLiftScore(2,(int)(robot.liftTicksPerInch*0), true);
                })
                .lineToSplineHeading(new Pose2d(-33.75,-15.75,Math.toRadians(135)))

                .UNSTABLE_addTemporalMarkerOffset(0.0, clawControl::openClaw)
                .waitSeconds(0.15)

                //CYCLE 1
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {liftTarget = clawControl.moveLiftGrab();})
                .splineToLinearHeading(new Pose2d(-70.0,-7,Math.toRadians(180)),Math.toRadians(170))

                .UNSTABLE_addTemporalMarkerOffset(0.25, clawControl::closeClaw)

                .UNSTABLE_addTemporalMarkerOffset(.45, () -> {liftTarget = clawControl.moveLiftScore(1,false);})
                .waitSeconds(0.6)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {liftTarget = clawControl.moveLiftScore(2,true);})
                .back(6)
                .splineTo(new Vector2d(-33,-15),Math.toRadians(-45))
                .UNSTABLE_addTemporalMarkerOffset(0.0, clawControl::openClaw)
                .waitSeconds(0.15)

                //CYCLE 2
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {liftTarget = clawControl.moveLiftGrab();})
                .splineToLinearHeading(new Pose2d(-71.5,-8.5,Math.toRadians(180)),Math.toRadians(170))

                .UNSTABLE_addTemporalMarkerOffset(0.25, clawControl::closeClaw)

                .UNSTABLE_addTemporalMarkerOffset(.45, () -> {liftTarget = clawControl.moveLiftScore(1,false);})
                .waitSeconds(0.6)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {liftTarget = clawControl.moveLiftScore(2,true);})
                .back(6)
                .splineTo(new Vector2d(-34,-16.5),Math.toRadians(-45))
                .UNSTABLE_addTemporalMarkerOffset(0.0, clawControl::openClaw)
               .waitSeconds(0.15)

                //CYCLE 3
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {liftTarget = clawControl.moveLiftGrab();})
                .splineToLinearHeading(new Pose2d(-72.5,-10.5,Math.toRadians(180)),Math.toRadians(170))

                .UNSTABLE_addTemporalMarkerOffset(0.25, clawControl::closeClaw)

                .UNSTABLE_addTemporalMarkerOffset(.45, () -> {liftTarget = clawControl.moveLiftScore(1,false);})
                .waitSeconds(0.6)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {liftTarget = clawControl.moveLiftScore(2,true);})
                .back(6)
                .splineTo(new Vector2d(-34,-19),Math.toRadians(-45))
                .UNSTABLE_addTemporalMarkerOffset(0.0, clawControl::openClaw)
                .waitSeconds(0.15)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {liftTarget = clawControl.moveLiftScore(0,false);})

//                .splineTo(new Vector2d(-44,-12),Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-44,-12,Math.toRadians(90)),Math.toRadians(135))
// park
                .lineToSplineHeading(new Pose2d(-18,-13,Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {stop();})
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
        camera.stopStreaming();

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

    public void stop(){
        drive.setMotorPowers(0,0,0,0);
     //   clawControl.runTo(robot.LIFT_BOTTOM);
        requestOpModeStop();
    }


}