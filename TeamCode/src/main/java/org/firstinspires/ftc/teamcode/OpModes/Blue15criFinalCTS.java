package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Autonomous(name = "Blue Auto CRI-CTS", group = "Competition")
@Disabled
public class Blue15criFinalCTS extends OpMode {
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
        //robot.autoLight.set(-1);

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

        Pose2d startPose = new Pose2d(params.startPoseX, params.startPoseY, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        trajectory1 = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0, clawControl::closeClaw)

                //lift claw while rotating
                .UNSTABLE_addTemporalMarkerOffset(.35, () -> {
                    liftTarget = clawControl.moveLiftScore(3,(int)(robot.liftTicksPerInch*3), true);
                })

                // drive forward to high junction rotating around the signal cone
                .splineToLinearHeading(new Pose2d(38,-44,Math.toRadians(90)),Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(44,1,Math.toRadians(-50)),Math.toRadians(90))
                .waitSeconds(0.25)
                // release the cone
                .UNSTABLE_addTemporalMarkerOffset(-0.25, clawControl::openClaw)

                //cycle 1
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {liftTarget = clawControl.moveLiftGrab();})
                // go to stack
                .splineToLinearHeading(new Pose2d(69.5,-5,Math.toRadians(0)),Math.toRadians(-30),
                        SampleMecanumDrive.getVelocityConstraint(40, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0.0, clawControl::closeClaw)

                // raise the lift to scoring position
                .UNSTABLE_addTemporalMarkerOffset(0.05, () -> {
                    liftTarget = clawControl.moveLiftScore(3,(int)(robot.liftTicksPerInch*3), true);
                })

                // head to high junction to score
                .splineToLinearHeading(new Pose2d(45,0,Math.toRadians(-30)),Math.toRadians(-30),
                        SampleMecanumDrive.getVelocityConstraint(40, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0.05, clawControl::openClaw)

                //cycle 2
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {liftTarget = clawControl.moveLiftGrab();})
                .splineToLinearHeading(new Pose2d(68.5,-5,Math.toRadians(0)),Math.toRadians(-30),
                        SampleMecanumDrive.getVelocityConstraint(40, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0.0, clawControl::closeClaw)

                // raise the lift to scoring position
                .UNSTABLE_addTemporalMarkerOffset(0.05, () -> {
                    liftTarget = clawControl.moveLiftScore(3,(int)(robot.liftTicksPerInch*3), true);
                })

                // head to high junction to score
                .splineToLinearHeading(new Pose2d(45,0,Math.toRadians(-30)),Math.toRadians(-30),
                        SampleMecanumDrive.getVelocityConstraint(40, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0.05, clawControl::openClaw)

                //cycle 3
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {liftTarget = clawControl.moveLiftGrab();})
                .splineToLinearHeading(new Pose2d(68.5,-6.5,Math.toRadians(0)),Math.toRadians(-30),
                        SampleMecanumDrive.getVelocityConstraint(40, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, clawControl::closeClaw)

                // raise the lift to scoring position
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    liftTarget = clawControl.moveLiftScore(3,(int)(robot.liftTicksPerInch*3), true);
                })

                // head to high junction to score
                .splineToLinearHeading(new Pose2d(45,0,Math.toRadians(-30)),Math.toRadians(-30),
                        SampleMecanumDrive.getVelocityConstraint(40, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, clawControl::openClaw)

                //cycle 4
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {liftTarget = clawControl.moveLiftGrab();})
                .splineToLinearHeading(new Pose2d(68.5,-6.5,Math.toRadians(0)),Math.toRadians(-30),
                        SampleMecanumDrive.getVelocityConstraint(40, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, clawControl::closeClaw)

                // raise the lift to scoring position
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    liftTarget = clawControl.moveLiftScore(3,(int)(robot.liftTicksPerInch*3), true);
                })

                .splineToLinearHeading(new Pose2d(45,0,Math.toRadians(-30)),Math.toRadians(-30),
                        SampleMecanumDrive.getVelocityConstraint(40, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, clawControl::openClaw)

                //cycle 5
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {liftTarget = clawControl.moveLiftGrab();})
                .splineToLinearHeading(new Pose2d(68.5,-7.5,Math.toRadians(0)),Math.toRadians(-30),
                        SampleMecanumDrive.getVelocityConstraint(40, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, clawControl::closeClaw)

                // raise the lift to scoring position
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    liftTarget = clawControl.moveLiftScore(3,(int)(robot.liftTicksPerInch*3), true);
                })

                .splineToLinearHeading(new Pose2d(45,0,Math.toRadians(-30)),Math.toRadians(-30),
                        SampleMecanumDrive.getVelocityConstraint(40, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, clawControl::openClaw)

                // Return lift to starting position
                .UNSTABLE_addTemporalMarkerOffset(0.25,() -> {liftTarget = clawControl.moveLiftScore(0,false);})

                // park
                .lineToSplineHeading(new Pose2d(36,-13,Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(-10,-13,Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(-10,-28,Math.toRadians(90)))

                // park before this
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {stop();})
                .build();

        trajectory2 = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0, clawControl::closeClaw)

                //lift claw while rotating
                .UNSTABLE_addTemporalMarkerOffset(.35, () -> {
                    liftTarget = clawControl.moveLiftScore(3,(int)(robot.liftTicksPerInch*3), true);
                })

                // drive forward to high junction rotating around the signal cone
                .splineToLinearHeading(new Pose2d(38,-44,Math.toRadians(90)),Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(44,1,Math.toRadians(-50)),Math.toRadians(90))
                .waitSeconds(0.25)
                // release the cone
                .UNSTABLE_addTemporalMarkerOffset(-0.25, clawControl::openClaw)

                //cycle 1
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {liftTarget = clawControl.moveLiftGrab();})
                // go to stack
                .splineToLinearHeading(new Pose2d(69.5,-5,Math.toRadians(0)),Math.toRadians(-30),
                        SampleMecanumDrive.getVelocityConstraint(40, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0.0, clawControl::closeClaw)

                // raise the lift to scoring position
                .UNSTABLE_addTemporalMarkerOffset(0.05, () -> {
                    liftTarget = clawControl.moveLiftScore(3,(int)(robot.liftTicksPerInch*3), true);
                })

                // head to high junction to score
                .splineToLinearHeading(new Pose2d(45,0,Math.toRadians(-30)),Math.toRadians(-30),
                        SampleMecanumDrive.getVelocityConstraint(40, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0.05, clawControl::openClaw)

                //cycle 2
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {liftTarget = clawControl.moveLiftGrab();})
                .splineToLinearHeading(new Pose2d(68.5,-5,Math.toRadians(0)),Math.toRadians(-30),
                        SampleMecanumDrive.getVelocityConstraint(40, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0.0, clawControl::closeClaw)

                // raise the lift to scoring position
                .UNSTABLE_addTemporalMarkerOffset(0.05, () -> {
                    liftTarget = clawControl.moveLiftScore(3,(int)(robot.liftTicksPerInch*3), true);
                })

                // head to high junction to score
                .splineToLinearHeading(new Pose2d(45,0,Math.toRadians(-30)),Math.toRadians(-30),
                        SampleMecanumDrive.getVelocityConstraint(40, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0.05, clawControl::openClaw)

                //cycle 3
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {liftTarget = clawControl.moveLiftGrab();})
                .splineToLinearHeading(new Pose2d(68.5,-6.5,Math.toRadians(0)),Math.toRadians(-30),
                        SampleMecanumDrive.getVelocityConstraint(40, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, clawControl::closeClaw)

                // raise the lift to scoring position
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    liftTarget = clawControl.moveLiftScore(3,(int)(robot.liftTicksPerInch*3), true);
                })

                // head to high junction to score
                .splineToLinearHeading(new Pose2d(45,0,Math.toRadians(-30)),Math.toRadians(-30),
                        SampleMecanumDrive.getVelocityConstraint(40, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, clawControl::openClaw)

                //cycle 4
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {liftTarget = clawControl.moveLiftGrab();})
                .splineToLinearHeading(new Pose2d(68.5,-6.5,Math.toRadians(0)),Math.toRadians(-30),
                        SampleMecanumDrive.getVelocityConstraint(40, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, clawControl::closeClaw)

                // raise the lift to scoring position
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    liftTarget = clawControl.moveLiftScore(3,(int)(robot.liftTicksPerInch*3), true);
                })

                .splineToLinearHeading(new Pose2d(45,0,Math.toRadians(-30)),Math.toRadians(-30),
                        SampleMecanumDrive.getVelocityConstraint(40, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, clawControl::openClaw)

                //cycle 5
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {liftTarget = clawControl.moveLiftGrab();})
                .splineToLinearHeading(new Pose2d(68.5,-7.5,Math.toRadians(0)),Math.toRadians(-30),
                        SampleMecanumDrive.getVelocityConstraint(40, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, clawControl::closeClaw)

                // raise the lift to scoring position
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    liftTarget = clawControl.moveLiftScore(3,(int)(robot.liftTicksPerInch*3), true);
                })

                .splineToLinearHeading(new Pose2d(45,0,Math.toRadians(-30)),Math.toRadians(-30),
                        SampleMecanumDrive.getVelocityConstraint(40, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, clawControl::openClaw)

                // Return lift to starting position
                .UNSTABLE_addTemporalMarkerOffset(0.25,() -> {liftTarget = clawControl.moveLiftScore(0,false);})

                // park
                .lineToSplineHeading(new Pose2d(36,-12,Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(14,-12,Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(14,-28,Math.toRadians(90)))

                // park before this
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {stop();})
                .build();

        trajectory3 = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0, clawControl::closeClaw)

                //lift claw while rotating
                .UNSTABLE_addTemporalMarkerOffset(.35, () -> {
                    liftTarget = clawControl.moveLiftScore(3,(int)(robot.liftTicksPerInch*3), true);
                })

                // drive forward to high junction rotating around the signal cone
                .splineToLinearHeading(new Pose2d(38,-44,Math.toRadians(90)),Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(44,1,Math.toRadians(-50)),Math.toRadians(90))
                .waitSeconds(0.25)
                // release the cone
                .UNSTABLE_addTemporalMarkerOffset(-0.25, clawControl::openClaw)

                //cycle 1
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {liftTarget = clawControl.moveLiftGrab();})
                // go to stack
                .splineToLinearHeading(new Pose2d(69.5,-5,Math.toRadians(0)),Math.toRadians(-30),
                        SampleMecanumDrive.getVelocityConstraint(40, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0.0, clawControl::closeClaw)

                // raise the lift to scoring position
                .UNSTABLE_addTemporalMarkerOffset(0.05, () -> {
                    liftTarget = clawControl.moveLiftScore(3,(int)(robot.liftTicksPerInch*3), true);
                })

                // head to high junction to score
                .splineToLinearHeading(new Pose2d(45,0,Math.toRadians(-30)),Math.toRadians(-30),
                        SampleMecanumDrive.getVelocityConstraint(40, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0.05, clawControl::openClaw)

                //cycle 2
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {liftTarget = clawControl.moveLiftGrab();})
                .splineToLinearHeading(new Pose2d(68.5,-5,Math.toRadians(0)),Math.toRadians(-30),
                        SampleMecanumDrive.getVelocityConstraint(40, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0.0, clawControl::closeClaw)

                // raise the lift to scoring position
                .UNSTABLE_addTemporalMarkerOffset(0.05, () -> {
                    liftTarget = clawControl.moveLiftScore(3,(int)(robot.liftTicksPerInch*3), true);
                })

                // head to high junction to score
                .splineToLinearHeading(new Pose2d(45,0,Math.toRadians(-30)),Math.toRadians(-30),
                        SampleMecanumDrive.getVelocityConstraint(40, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0.05, clawControl::openClaw)

                //cycle 3
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {liftTarget = clawControl.moveLiftGrab();})
                .splineToLinearHeading(new Pose2d(68.5,-6.5,Math.toRadians(0)),Math.toRadians(-30),
                        SampleMecanumDrive.getVelocityConstraint(40, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, clawControl::closeClaw)

                // raise the lift to scoring position
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    liftTarget = clawControl.moveLiftScore(3,(int)(robot.liftTicksPerInch*3), true);
                })

                // head to high junction to score
                .splineToLinearHeading(new Pose2d(45,0,Math.toRadians(-30)),Math.toRadians(-30),
                        SampleMecanumDrive.getVelocityConstraint(40, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, clawControl::openClaw)

                //cycle 4
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {liftTarget = clawControl.moveLiftGrab();})
                .splineToLinearHeading(new Pose2d(68.5,-6.5,Math.toRadians(0)),Math.toRadians(-30),
                        SampleMecanumDrive.getVelocityConstraint(40, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, clawControl::closeClaw)

                // raise the lift to scoring position
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    liftTarget = clawControl.moveLiftScore(3,(int)(robot.liftTicksPerInch*3), true);
                })

                .splineToLinearHeading(new Pose2d(45,0,Math.toRadians(-30)),Math.toRadians(-30),
                        SampleMecanumDrive.getVelocityConstraint(40, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, clawControl::openClaw)

                //cycle 5
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {liftTarget = clawControl.moveLiftGrab();})
                .splineToLinearHeading(new Pose2d(68.5,-7.5,Math.toRadians(0)),Math.toRadians(-30),
                        SampleMecanumDrive.getVelocityConstraint(40, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, clawControl::closeClaw)

                // raise the lift to scoring position
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    liftTarget = clawControl.moveLiftScore(3,(int)(robot.liftTicksPerInch*3), true);
                })

                .splineToLinearHeading(new Pose2d(45,0,Math.toRadians(-30)),Math.toRadians(-30),
                        SampleMecanumDrive.getVelocityConstraint(40, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, clawControl::openClaw)

                // Return lift to starting position
                .UNSTABLE_addTemporalMarkerOffset(0.25,() -> {liftTarget = clawControl.moveLiftScore(0,false);})

                // park
                .lineToSplineHeading(new Pose2d(36,-20,Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(36,-28,Math.toRadians(90)))

                // park before this
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
