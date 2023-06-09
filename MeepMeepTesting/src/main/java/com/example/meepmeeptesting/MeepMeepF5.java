package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepF5 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15.37)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(34, -63, Math.toRadians(90)))
                                .splineToConstantHeading(new Vector2d(12, -55), Math.toRadians(90))
                                .lineToConstantHeading(new Vector2d(12, -42))
                                .addDisplacementMarker(()->{
                                    //enhancedDriver.moveArm(3);
                                })
                                .lineToSplineHeading(new Pose2d(12, -30, Math.toRadians(180)))
                                //.lineToConstantHeading(new Vector2d(12, -24))
                                .splineToConstantHeading(new Vector2d(5, -26), Math.toRadians(180))
                                .addDisplacementMarker(()->{
                                    //enhancedDriver.moveArm(2);
                                })
                                .addDisplacementMarker(()->{
                                    //enhancedDriver.turnGrabber(2);
                                })
                                .waitSeconds(1)
                                .splineToConstantHeading(new Vector2d(12, -18), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(12, -12, Math.toRadians(270)), Math.toRadians(0))
                                //.lineTo(calculateTargetPositions(StartTile.F5, parkPosition).vec())
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}