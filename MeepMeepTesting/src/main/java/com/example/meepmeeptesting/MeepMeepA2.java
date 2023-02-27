package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepA2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15.37)
                .followTrajectorySequence(drive ->
                    drive.trajectorySequenceBuilder(new Pose2d(-31, 63, Math.toRadians(-90)))
                            .splineToConstantHeading(new Vector2d(-24, 61), Math.toRadians(0))
                            .lineToConstantHeading(new Vector2d(-18, 61))
                            .splineToConstantHeading(new Vector2d(-12, 55), Math.toRadians(-90))
                            .lineToSplineHeading(new Pose2d(-12, 30, Math.toRadians(0)))
                            .splineToConstantHeading(new Vector2d(-9, 27.5), Math.toRadians(0))
                            .splineToConstantHeading(new Vector2d(-12, 21), Math.toRadians(-90))
                            .splineToConstantHeading(new Vector2d(-24, 12), Math.toRadians(-180))
                            .lineToSplineHeading(new Pose2d(-50, 12, Math.toRadians(-180)))
                            .lineToConstantHeading(new Vector2d(-63, 12))

                            .lineToConstantHeading(new Vector2d(-55, 12))
                            .lineToSplineHeading(new Pose2d(-30, 12, Math.toRadians(-90)))
                            .splineToConstantHeading(new Vector2d(-24, 9), Math.toRadians(-90))
                            .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}