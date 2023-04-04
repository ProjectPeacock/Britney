package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Libs.AutoParams;
import org.firstinspires.ftc.teamcode.Libs.LiftControlClass;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Blue Cycle Auto April Tags", group = "Competition")
public class BlueCycleAutoAprTags extends LinearOpMode {
    FtcDashboard dashboard;
    TelemetryPacket dashTelemetry = new TelemetryPacket();
    public static double preloadX = 28.5;
    public static double preloadY = -30;
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
    private LinearOpMode myOpmode=this;
    LiftControlClass clawControl = new LiftControlClass(robot,myOpmode);

    //init params
    AutoParams params = new AutoParams();

    int parkPosition = 2;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });


        robot.init(hardwareMap);
        robot.servoArm.setPosition(robot.SERVO_ARM_INTAKE);
        dashboard = FtcDashboard.getInstance();
        TelemetryPacket dashTelemetry = new TelemetryPacket();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose= new Pose2d(params.startPoseX,params.startPoseY,Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence untilCycle = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0, clawControl::closeClaw)
                .waitSeconds(0.125)
                .UNSTABLE_addTemporalMarkerOffset(0.65,()->{clawControl.lowerAligner();})
                .UNSTABLE_addTemporalMarkerOffset(0.65,()->{clawControl.moveLiftScore(2);})
                .splineTo(new Vector2d(preloadX,preloadY),Math.toRadians(120))
                .UNSTABLE_addTemporalMarkerOffset(0.25,()->{clawControl.moveLiftScore(2,75);})
                .UNSTABLE_addTemporalMarkerOffset(0.35, clawControl::openClaw)
                .waitSeconds(0.35)
                .back(6)
                .UNSTABLE_addTemporalMarkerOffset(-0.125,()->{clawControl.moveLiftGrab();})
                .turn(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(63,-8,Math.toRadians(0)),Math.toRadians(0))

                .UNSTABLE_addTemporalMarkerOffset(0, clawControl::closeClaw)
                .waitSeconds(0.35)
                .build();

        TrajectorySequence cycleMid1 = drive.trajectorySequenceBuilder(untilCycle.end())
                .UNSTABLE_addTemporalMarkerOffset(0,()->{clawControl.moveLiftScore(1,50);})
                //.UNSTABLE_addTemporalMarkerOffset(0.25,()->{clawControl.moveLiftScore(0);})
                .back(12)
                .UNSTABLE_addTemporalMarkerOffset(0.65,()->{clawControl.lowerAligner();})
                .UNSTABLE_addTemporalMarkerOffset(0.75,()->{clawControl.moveLiftScore(2);})
                .splineToSplineHeading(new Pose2d(35.5,-15.5,Math.toRadians(220)),Math.toRadians(190))
                .forward(6.5)
                .UNSTABLE_addTemporalMarkerOffset(0.25,()->{clawControl.moveLiftScore(2,75);})
                .UNSTABLE_addTemporalMarkerOffset(0.5, clawControl::openClaw)
                .waitSeconds(0.5)
                .back(4)
                .UNSTABLE_addTemporalMarkerOffset(0.5, clawControl::moveLiftGrab)
                .splineToSplineHeading(new Pose2d(62.5,-8,Math.toRadians(0)),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0.35,clawControl::closeClaw)
                .waitSeconds(0.35)
                .back(1)

                .build();

        TrajectorySequence cycleMid2 = drive.trajectorySequenceBuilder(cycleMid1.end())
                .UNSTABLE_addTemporalMarkerOffset(0,()->{clawControl.moveLiftScore(1,50);})
                //.UNSTABLE_addTemporalMarkerOffset(0.25,()->{clawControl.moveLiftScore(0);})
                .back(12)
                .UNSTABLE_addTemporalMarkerOffset(0.65,()->{clawControl.lowerAligner();})
                .UNSTABLE_addTemporalMarkerOffset(0.75,()->{clawControl.moveLiftScore(2);})
                .splineToSplineHeading(new Pose2d(35.5,-15.5,Math.toRadians(220)),Math.toRadians(190))
                .forward(6.5)
                .UNSTABLE_addTemporalMarkerOffset(0.25,()->{clawControl.moveLiftScore(2,75);})
                .UNSTABLE_addTemporalMarkerOffset(0.5, clawControl::openClaw)
                .waitSeconds(0.5)
                .back(4)
                .UNSTABLE_addTemporalMarkerOffset(0.5, clawControl::moveLiftGrab)
                .splineToSplineHeading(new Pose2d(62,-6.5,Math.toRadians(0)),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0.35,clawControl::closeClaw)
                .waitSeconds(0.35)
                .back(1)
                .build();

        TrajectorySequence finalMid = drive.trajectorySequenceBuilder(cycleMid1.end())
                .UNSTABLE_addTemporalMarkerOffset(0,()->{clawControl.moveLiftScore(1,50);})
                //.UNSTABLE_addTemporalMarkerOffset(0.25,()->{clawControl.moveLiftScore(0);})
                .back(11)
                .UNSTABLE_addTemporalMarkerOffset(0.25,()->{clawControl.lowerAligner();})
                .UNSTABLE_addTemporalMarkerOffset(0.75,()->{clawControl.moveLiftScore(2);})
                .splineToSplineHeading(new Pose2d(35.5,-13.5,Math.toRadians(220)),Math.toRadians(190))
                .forward(6.5)
                .UNSTABLE_addTemporalMarkerOffset(0.25,()->{clawControl.moveLiftScore(2,75);})
                .UNSTABLE_addTemporalMarkerOffset(0.5, clawControl::openClaw)
                .waitSeconds(0.5)
                .back(8)
                .UNSTABLE_addTemporalMarkerOffset(0.25,()->{clawControl.moveLiftScore(0);})
                .splineToSplineHeading(new Pose2d(36,-13.5,Math.toRadians(90)),Math.toRadians(220))
                .UNSTABLE_addTemporalMarkerOffset(0.25,()->{robot.servoArm.setPosition(robot.SERVO_ARM_DUNK);})
                .build();

        /*

        TrajectorySequence finalHigh = drive.trajectorySequenceBuilder(cycleHigh.end())
                .UNSTABLE_addTemporalMarkerOffset(0,()->{clawControl.moveLiftScore(3);})
                .back(6)
                .splineToSplineHeading(new Pose2d(30,-6.5,Math.toRadians(130)),Math.toRadians(220))
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(-0.25,()->{clawControl.openClaw();})
                .back(1)
                .build();

         */

        TrajectorySequence park1 = drive.trajectorySequenceBuilder(finalMid.end())
                .strafeLeft(22)
                .back(1.5)
                .UNSTABLE_addTemporalMarkerOffset(0.25,()->{robot.servoArm.setPosition(robot.SERVO_ARM_DUNK);})

                //.splineToSplineHeading(new Pose2d(12,-14,Math.toRadians(90)),Math.toRadians(180))
                .build();

        TrajectorySequence park3 = drive.trajectorySequenceBuilder(finalMid.end())
                .strafeRight(22)
                .back(1.5)
                .UNSTABLE_addTemporalMarkerOffset(0.25,()->{robot.servoArm.setPosition(robot.SERVO_ARM_DUNK);})

                //.splineToSplineHeading(new Pose2d(60,-14,Math.toRadians(90)),Math.toRadians(0))
                .build();

        while(!isStopRequested() && !opModeIsActive()) {
            robot.autoLight.set(-1);

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0) {
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
            }
                else
            {
                telemetry.addLine("Don't see any tags of interest :(");
            }
            telemetry.update();
        }  // end of while

        waitForStart();

        dashTelemetry.put("01 - IMU Angle X = ", robot.imu.getAngles()[0]);
        dashTelemetry.put("02 - IMU Angle Y = ", robot.imu.getAngles()[1]);
        dashTelemetry.put("03 - IMU Angle Z = ", robot.imu.getAngles()[2]);

        dashTelemetry.put("06 - Claw Value = ", robot.servoGrabber.getPosition());
        dashTelemetry.put("07 - GP1.Button.A = ", "RESET LIFT");
        dashTelemetry.put("08 - GP1.Button.B = ", "LIFT LOW JUNCTION");
        dashTelemetry.put("09 - GP1.Button.X = ", "LIFT MID JUNCTION");
        dashTelemetry.put("10 - GP1.Button.Y = ", "LIFT HIGH JUNCTION");
        dashTelemetry.put("11 - GP2.Button.A = ", "Custom Position - program stack cone levels");
        dashboard.sendTelemetryPacket(dashTelemetry);


        robot.autoLight.set(0);
        if(isStopRequested()) return;
        //score preload
        drive.followTrajectorySequence(untilCycle);
        drive.followTrajectorySequence(cycleMid1);
        //drive.followTrajectorySequence(cycleMid2);
        //drive.followTrajectorySequence(cycleMid3);

        /*
        for(int i=0;i<params.numHighCycles;i++){
            drive.followTrajectorySequence(cycleHigh);
        }
        */
        if (myOpmode.time < 5){
            drive.followTrajectorySequence(finalMid);
        }


        if(parkPosition==1){
            drive.followTrajectorySequence(park1);
        }else if(parkPosition==3){
            drive.followTrajectorySequence(park3);
        }
    }

}
