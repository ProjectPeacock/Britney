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
                        drive.trajectorySequenceBuilder(new Pose2d(-38.25,-63, Math.toRadians(90)))
                                //drive to mid pole
                                .splineToLinearHeading(new Pose2d(-40,-44,Math.toRadians(90)),Math.toRadians(90))

                                .splineToLinearHeading(new Pose2d(-21,-8,Math.toRadians(225)),Math.toRadians(10))                                //.back(1)

                                .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}