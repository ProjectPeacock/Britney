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
                .setConstraints(45, 65, 17.927212540948645, Math.toRadians(360), 11.42)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35.25,-64, Math.toRadians(90)))
                                //drive to mid pole
                                .waitSeconds(0.125)
                                .splineTo(new Vector2d(27,-29.5),Math.toRadians(120))
                                .waitSeconds(0.35)
                                .back(3)
                                .turn(Math.toRadians(-45))
                                .splineToLinearHeading(new Pose2d(55.5,-6,Math.toRadians(0)),Math.toRadians(0))
                                .forward(6)
                                .waitSeconds(0.35)

                                //low cycle code
                                .back(1)
                                .splineToConstantHeading(new Vector2d(36,-24),Math.toRadians(0))
                                .forward(4)
                                .waitSeconds(0.35)
                                .back(2)
                                .splineToConstantHeading(new Vector2d(55.5,-6),Math.toRadians(0))
                                .forward(6)

                                .back(1)
                                .splineToConstantHeading(new Vector2d(36,-24),Math.toRadians(0))
                                .forward(4)
                                .waitSeconds(0.35)
                                .back(2)
                                .splineToSplineHeading(new Pose2d(36,-14,Math.toRadians(90)),Math.toRadians(90))
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