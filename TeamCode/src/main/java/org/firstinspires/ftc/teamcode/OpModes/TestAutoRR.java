package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Test Auto RR", group = "Competition")
@Disabled
public class TestAutoRR extends OpMode {
    /*

    OPMODE MAP - PLEASE READ BEFORE EDITING

    This opMode uses TrajectorySequences from RoadRunner. They are made to be run back to back.
    The order of operations is: untilCycle -> firstCycle -> cycles2to4 (repeated 3 times) -> highCycle -> park

    Each parking position is its own TrajectorySequence. They are all made to be run following highCycle.

    All values for target position and heading come from AutoParams.java.

     */

    //lift control init

//    public HWProfile robot = new HWProfile();
//    private OpMode myOpmode=this;
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//    private int parkPos = 1;
//    public DcMotorEx motorLiftF = null;
//    public DcMotorEx motorLiftR = null;


    public void init() {
        /*
        motorLiftF = hardwareMap.get(DcMotorEx.class, "motorLiftFront");
        motorLiftF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorLiftF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLiftF.setPower(0);

        motorLiftR = hardwareMap.get(DcMotorEx.class, "motorLiftRear");
        motorLiftR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorLiftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLiftR.setPower(0);

        robot.init(hardwareMap);

        //init params
        AutoParams params = new AutoParams();

        // start up the shooter control program which runs in parallel
        LiftThread liftControl = new LiftThread(robot, motorLiftF, motorLiftR, params);
        Thread lift = new Thread(liftControl);

        lift.start();

        Pose2d startPose= new Pose2d(params.startPoseX,params.startPoseY,Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence untilCycle = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(28.5,-31),Math.toRadians(120))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    liftControl.moveLiftScore(2);
                })
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(-0.25,()->{
                    liftControl.openClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.25,()->{
                    liftControl.moveLiftGrab();
                })
                .turn(Math.toRadians(-60))
                .splineToLinearHeading(new Pose2d(60,-12,Math.toRadians(0)),Math.toRadians(0))
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(-0.25,()->{
                    liftControl.closeClaw();
                })
                .build();

        TrajectorySequence cycleMid = drive.trajectorySequenceBuilder(untilCycle.end())
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    liftControl.moveLiftScore(2);
                })
                .back(6)
                .splineToSplineHeading(new Pose2d(30,-17.5,Math.toRadians(220)),Math.toRadians(190))
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(-0.25,()->{
                    liftControl.openClaw();
                })
                .back(1)
                .UNSTABLE_addTemporalMarkerOffset(0.25,()->{
                    liftControl.moveLiftGrab();
                })
                .splineToSplineHeading(new Pose2d(60,-12,Math.toRadians(0)),Math.toRadians(0))
                .build();

        TrajectorySequence cycleHigh = drive.trajectorySequenceBuilder(cycleMid.end())
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    liftControl.moveLiftScore(3);
                })
                .back(6)
                .splineToSplineHeading(new Pose2d(30,-6.5,Math.toRadians(130)),Math.toRadians(17220))
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(-0.25,()->{
                    liftControl.openClaw();
                })
                .back(1)
                .UNSTABLE_addTemporalMarkerOffset(0.25,()->{
                    liftControl.moveLiftGrab();
                })
                .splineToSplineHeading(new Pose2d(60,-12,Math.toRadians(0)),Math.toRadians(0))
                .build();

        TrajectorySequence finalHigh = drive.trajectorySequenceBuilder(cycleHigh.end())
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    liftControl.moveLiftScore(3);
                })
                .back(6)
                .splineToSplineHeading(new Pose2d(30,-6.5,Math.toRadians(130)),Math.toRadians(17220))
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(-0.25,()->{
                    liftControl.openClaw();
                })
                .back(1)
                .build();

        TrajectorySequence park1 = drive.trajectorySequenceBuilder(finalHigh.end())
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    liftControl.moveLiftScore(0);
                })
                .splineToSplineHeading(new Pose2d(12,-12,Math.toRadians(90)),Math.toRadians(180))
                .build();

        TrajectorySequence park2 = drive.trajectorySequenceBuilder(finalHigh.end())
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    liftControl.moveLiftScore(0);
                })
                .splineToSplineHeading(new Pose2d(36,-12,Math.toRadians(90)),Math.toRadians(130))
                .build();

        TrajectorySequence park3 = drive.trajectorySequenceBuilder(finalHigh.end())
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    liftControl.moveLiftScore(0);
                })
                .splineToSplineHeading(new Pose2d(60,-12,Math.toRadians(90)),Math.toRadians(0))
                .build();



        //score preload
        drive.followTrajectorySequenceAsync(untilCycle);
        for(int i=0;i<params.numMidCycles;i++){
            drive.followTrajectorySequenceAsync(cycleMid);
        }
        for(int i=0;i<params.numHighCycles;i++){
            drive.followTrajectorySequenceAsync(cycleHigh);
        }

        drive.followTrajectorySequenceAsync(finalHigh);

        if(parkPos==1){
            drive.followTrajectorySequenceAsync(park1);
        }else if(parkPos==2){
            drive.followTrajectorySequenceAsync(park2);
        }else{
            drive.followTrajectorySequenceAsync(park3);
        }

         */

    }
    public void loop(){
//        drive.update();
    }

}
