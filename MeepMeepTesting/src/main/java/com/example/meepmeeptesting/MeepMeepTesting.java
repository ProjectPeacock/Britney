package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.io.IOException;
import java.net.URL;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) throws IOException {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(65, 60, 15, 15, 16.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36,-63, Math.toRadians(90)))
                                //drive to mid pole
                                .forward(6)
                                .turn(Math.toRadians(90))

                                //line to push signal
                                .lineToSplineHeading(new Pose2d(-36,-6,Math.toRadians(180)))

                                //line to score preload on mid junction
                                .lineToSplineHeading(new Pose2d(-30,-18,Math.toRadians(135)))

                                //CYCLE 1
                                //spline to pick up cone from stack
                                .splineToLinearHeading(new Pose2d(-67.5,-8,Math.toRadians(180)),Math.toRadians(170))

                                //spline to score cone on mid junction
                                .back(6)
                                .splineToSplineHeading(new Pose2d(-30,-18,Math.toRadians(135)),Math.toRadians(-50))

                                //CYCLE 2
                                //spline to pick up cone from stack
                                .splineToLinearHeading(new Pose2d(-67.5,-8,Math.toRadians(180)),Math.toRadians(170))

                                //spline to score cone on mid junction
                                .back(6)
                                .splineToSplineHeading(new Pose2d(-30,-18,Math.toRadians(135)),Math.toRadians(-50))

                                //CYCLE 3
                                //spline to pick up cone from stack
                                .splineToLinearHeading(new Pose2d(-67.5,-8,Math.toRadians(180)),Math.toRadians(170))

                                //spline to score cone on mid junction
                                .back(6)
                                .splineToSplineHeading(new Pose2d(-30,-18,Math.toRadians(135)),Math.toRadians(-50))

                                //CYCLE 4
                                //spline to pick up cone from stack
                                .splineToLinearHeading(new Pose2d(-67.5,-8,Math.toRadians(180)),Math.toRadians(170))

                                //spline to score cone on mid junction
                                .back(6)
                                .splineToSplineHeading(new Pose2d(-30,-18,Math.toRadians(135)),Math.toRadians(-50))

                                //CYCLE 5
                                //spline to pick up cone from stack
                                .splineToLinearHeading(new Pose2d(-67.5,-8,Math.toRadians(180)),Math.toRadians(170))

                                //spline to score cone on mid junction
                                .back(6)
                                .splineToSplineHeading(new Pose2d(-30,-18,Math.toRadians(135)),Math.toRadians(-50))

                                //PARK
                                .lineToSplineHeading(new Pose2d(-36,-12,Math.toRadians(90)))
                                .strafeLeft(24)
                                .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}