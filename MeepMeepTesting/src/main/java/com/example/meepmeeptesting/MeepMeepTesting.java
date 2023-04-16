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
                .setConstraints(90, 90, 17.927212540948645, Math.toRadians(360), 11.42)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35.25,-64, Math.toRadians(90)))
                                //drive to mid pole
                                .waitSeconds(0.125)
                                .splineTo(new Vector2d(27.5,-27),Math.toRadians(120))
                                .waitSeconds(0.35)
                                .back(10)
                                .turn(Math.toRadians(-45))
                                .splineToLinearHeading(new Pose2d(69.5,-10,Math.toRadians(0)),Math.toRadians(0))
                                .waitSeconds(0.35)

                                //cycle 1
                                .back(12)
                                .splineToSplineHeading(new Pose2d(35.5,-13,Math.toRadians(30)),Math.toRadians(-170))
                                .waitSeconds(0.35)
                                .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}