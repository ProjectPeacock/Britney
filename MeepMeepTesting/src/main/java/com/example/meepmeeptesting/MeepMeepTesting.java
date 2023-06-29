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
                        drive.trajectorySequenceBuilder(new Pose2d(-38,-63, Math.toRadians(90)))
                                //drive to mid pole
                                .lineToSplineHeading(new Pose2d(-36,-6,Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(-30,-12,Math.toRadians(225)))
                                .lineToSplineHeading(new Pose2d(-19,-12,Math.toRadians(225)))

                                .splineToLinearHeading(new Pose2d(-60.0,-18.5,Math.toRadians(180)),Math.toRadians(190))



                                .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}