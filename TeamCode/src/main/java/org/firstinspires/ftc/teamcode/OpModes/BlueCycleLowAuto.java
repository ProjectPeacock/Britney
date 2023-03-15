package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.AutoParams;
import org.firstinspires.ftc.teamcode.Libs.LiftControlClass;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;

@Autonomous(name = "Blue Low Cycle Auto", group = "Competition")
public class BlueCycleLowAuto extends LinearOpMode {
    FtcDashboard dashboard;
    TelemetryPacket dashTelemetry = new TelemetryPacket();
    public static double preloadX = 28.5;
    public static double preloadY = -31;
    private String configFile="autoGyroValue.txt";
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

    //TFOD init
    private static final String TFOD_MODEL_ASSET = robot.tfliteFileName;
    private static final String[] LABELS = {
            "circle",
            "star",
            "triangle"
    };
    private static final String VUFORIA_KEY =
            "AfHl2GP/////AAABmeJc93xOhk1MvZeKbP5E43taYJ6kodzkhsk5wOLGwZI3wxf7v1iTx2Mem/VZSEtpxb3U2fMO7n0EUxSeHRWhOXeX16dMFcjfalezjo3ZkzBuG/y2r4kgLwKs4APyAIClBAon+tf/W/4NkTkYuHGo8zZ0slH/iBpqxvblpNURsG5h4VxPFgF5D/FIfmjnddzQpa4cGarle/Zvuah6q2orUswun31P6ZLuIJvdOIQf7o/ruoRygsSXfVYc35w+Xwm+bwjpZUNzHHYvRNrp0HNWC3Fr2hd0TqWKIIYlCoHj0m5OKX22Ris23V8PdKM/i4/ZIy8JewJXetv1rERC5bfHmUXCS4Rl7RjR+ZscQ5aA0nr8";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    double parkPosition = 2;

    @Override
    public void runOpMode() {
        initVuforia();
        initTfod();
        robot.init(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        TelemetryPacket dashTelemetry = new TelemetryPacket();


        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose= new Pose2d(params.startPoseX,params.startPoseY,Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence untilCycle = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0, clawControl::closeClaw)
                .waitSeconds(0.125)
                .UNSTABLE_addTemporalMarkerOffset(0.85,()->{clawControl.moveLiftScore(2);})
                .splineTo(new Vector2d(preloadX,preloadY),Math.toRadians(120))

                .UNSTABLE_addTemporalMarkerOffset(0.35, clawControl::openClaw)
                .waitSeconds(0.35)
                .back(3)
                .UNSTABLE_addTemporalMarkerOffset(-0.125,()->{clawControl.moveLiftGrab();})
                .turn(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(62,-8,Math.toRadians(0)),Math.toRadians(0))

                .UNSTABLE_addTemporalMarkerOffset(0, clawControl::closeClaw)
                .waitSeconds(0.35)
                .build();

        TrajectorySequence cycleMid = drive.trajectorySequenceBuilder(untilCycle.end())
                .UNSTABLE_addTemporalMarkerOffset(0,()->{clawControl.moveLiftScore(1,50);})
                //.UNSTABLE_addTemporalMarkerOffset(0.25,()->{clawControl.moveLiftScore(0);})
                .back(12.5)
                .UNSTABLE_addTemporalMarkerOffset(0.75,()->{clawControl.moveLiftScore(1);})
                .turn(Math.toRadians(-90))
                .forward(4)
                .UNSTABLE_addTemporalMarkerOffset(0.5, clawControl::openClaw)
                .waitSeconds(0.5)
                .back(2)
                .UNSTABLE_addTemporalMarkerOffset(0.25, clawControl::moveLiftGrab)
                .lineToSplineHeading(new Pose2d(57.5,-8,Math.toRadians(0)))
                .forward(2.5)
                .UNSTABLE_addTemporalMarkerOffset(0,clawControl::closeClaw)
                .waitSeconds(0.5)
                .build();

        /*
        TrajectorySequence cycleHigh = drive.trajectorySequenceBuilder(cycleMid.end())
                .UNSTABLE_addTemporalMarkerOffset(0,()->{clawControl.moveLiftScore(3);})
                .back(6)
                .splineToSplineHeading(new Pose2d(30,-6.5,Math.toRadians(130)),Math.toRadians(220))
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(-0.25, clawControl::openClaw)
                .back(1)
                .UNSTABLE_addTemporalMarkerOffset(0.25, clawControl::moveLiftGrab)
                .splineToSplineHeading(new Pose2d(60,-12,Math.toRadians(0)),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0,clawControl::closeClaw)
                .waitSeconds(0.5)
                .build();

         */

        TrajectorySequence finalMid = drive.trajectorySequenceBuilder(cycleMid.end())
                .UNSTABLE_addTemporalMarkerOffset(0,()->{clawControl.moveLiftScore(1,50);})
                //.UNSTABLE_addTemporalMarkerOffset(0.25,()->{clawControl.moveLiftScore(0);})
                .back(15)
                .UNSTABLE_addTemporalMarkerOffset(0.75,()->{clawControl.moveLiftScore(1);})
                .turn(Math.toRadians(-90))
                .forward(4)
                .UNSTABLE_addTemporalMarkerOffset(0.5, clawControl::openClaw)
                .waitSeconds(0.5)
                .back(3)
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
                .UNSTABLE_addTemporalMarkerOffset(0,()->{clawControl.moveLiftScore(0);})
                .splineToSplineHeading(new Pose2d(12,-14,Math.toRadians(90)),Math.toRadians(180))
                .build();

        TrajectorySequence park2 = drive.trajectorySequenceBuilder(finalMid.end())
                .UNSTABLE_addTemporalMarkerOffset(0,()->{clawControl.moveLiftScore(0);})
                .splineToSplineHeading(new Pose2d(36,-14,Math.toRadians(90)),Math.toRadians(130))
                .build();

        TrajectorySequence park3 = drive.trajectorySequenceBuilder(finalMid.end())
                .UNSTABLE_addTemporalMarkerOffset(0,()->{clawControl.moveLiftScore(0);})
                .splineToSplineHeading(new Pose2d(60,-14,Math.toRadians(90)),Math.toRadians(0))
                .build();

        while(!isStopRequested() && !opModeIsActive()) {
            robot.autoLight.set(-1);
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Objects Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display image position/size information for each one
                    // Note: "Image number" refers to the randomized image orientation/number
                    for (Recognition recognition : updatedRecognitions) {
                        double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                        double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                        double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                        double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                        telemetry.addData(""," ");
                        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                        telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                        telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);

                        if(recognition.getLabel() == "circle"){
                            parkPosition = 1;
                        } else if(recognition.getLabel() == "star" ){
                            parkPosition = 3;
                        } else parkPosition = 2;
                    }
                    telemetry.update();
                }
            }

        }  // end of while

        waitForStart();

        dashTelemetry.put("01 - IMU Angle X = ", robot.imu.getAngles()[0]);
        dashTelemetry.put("02 - IMU Angle Y = ", robot.imu.getAngles()[1]);
        dashTelemetry.put("03 - IMU Angle Z = ", robot.imu.getAngles()[2]);
        dashTelemetry.put("04 - Lift Front Encoder Value = ", robot.motorLiftFront.getCurrentPosition());
        dashTelemetry.put("05 - Lift Rear Encoder Value = ", robot.motorLiftRear.getCurrentPosition());
        dashTelemetry.put("06 - Claw Value = ", robot.servoGrabber.getPosition());
        dashTelemetry.put("07 - GP1.Button.A = ", "RESET LIFT");
        dashTelemetry.put("08 - GP1.Button.B = ", "LIFT LOW JUNCTION");
        dashTelemetry.put("09 - GP1.Button.X = ", "LIFT MID JUNCTION");
        dashTelemetry.put("10 - GP1.Button.Y = ", "LIFT HIGH JUNCTION");
        dashTelemetry.put("11 - GP2.Button.A = ", "Custom Position - program stack cone levels");
        dashboard.sendTelemetryPacket(dashTelemetry);


        robot.autoLight.set(0);
        if(isStopRequested()) return;

        robot.servoAlign.setPosition(robot.SERVO_ALIGN_UP);
        //score preload
        drive.followTrajectorySequence(untilCycle);
        for(int i=0;i<2;i++){
            drive.followTrajectorySequence(cycleMid);
        }

        /*
        for(int i=0;i<params.numHighCycles;i++){
            drive.followTrajectorySequence(cycleHigh);
        }
        */

        drive.followTrajectorySequence(finalMid);

        if(parkPosition==1){
            drive.followTrajectorySequence(park1);
        }else if(parkPosition==2){
            drive.followTrajectorySequence(park2);
        }else{
            drive.followTrajectorySequence(park3);
        }
    }
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }


    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.6f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
}
