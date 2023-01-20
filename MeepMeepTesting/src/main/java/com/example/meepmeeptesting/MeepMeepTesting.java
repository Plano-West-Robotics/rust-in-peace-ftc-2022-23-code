package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528, Math.toRadians(181.14757590361444), Math.toRadians(181.14757590361444), 16.6)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(38, 63, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(36, 60, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(12, 60, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(12, 12, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(24.5, 12, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(24.5, 7, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(24.5, 7.001, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(24.5, 12, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(32, 12, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(40, 12, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(62, 12, Math.toRadians(0)))
//                                .lineToLinearHeading(new Pose2d(-12, 60, Math.toRadians(0)))
//                                .lineToLinearHeading(new Pose2d(-12, 24, Math.toRadians(0)))
//                                .lineToLinearHeading(new Pose2d(-9, 24, Math.toRadians(0)))
//                                .lineToLinearHeading(new Pose2d(-12, 24, Math.toRadians(0)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}