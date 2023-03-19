package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepA5 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15.37)
                .followTrajectorySequence(drive ->
                    drive.trajectorySequenceBuilder(new Pose2d(38, 63, Math.toRadians(-90)))
                            .splineToConstantHeading(new Vector2d(30, 61), Math.toRadians(-180))
                            .lineToConstantHeading(new Vector2d(18, 61))

                            .splineToConstantHeading(new Vector2d(12, 50), Math.toRadians(-90))
                            .lineToConstantHeading(new Vector2d(12, 20))
                            .splineToConstantHeading(new Vector2d(18, 14), Math.toRadians(0))
                            .splineToConstantHeading(new Vector2d(22.5, 10), Math.toRadians(-90))



                            .splineToConstantHeading(new Vector2d(24, 12), 0)
                            .lineToConstantHeading(new Vector2d(36, 12))
                            .turn(Math.toRadians(90))
                            .lineToConstantHeading(new Vector2d(62, 12))


                            .lineToConstantHeading(new Vector2d(36, 12))
                            .turn(Math.toRadians(-90))
                            .lineToConstantHeading(new Vector2d(30, 12))
                            //drop location
                            .splineToConstantHeading(new Vector2d(25, 12-4.5), Math.toRadians(-90))

                            .splineToConstantHeading(new Vector2d(30, 12), 0)
                            .turn(Math.toRadians(90))
                            .lineToConstantHeading(new Vector2d(62, 12))



                            .lineToConstantHeading(new Vector2d(36, 12))
                            .turn(Math.toRadians(-90))
                            .lineToConstantHeading(new Vector2d(30, 12))
                            .splineToConstantHeading(new Vector2d(25, 12-4.5), Math.toRadians(-90))

                            .lineToConstantHeading(new Vector2d(26, 12))


                            .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}