package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(16, 16)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 10)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(15, -62.5, Math.toRadians(90)))

                .strafeToLinearHeading(new Vector2d(4, -30), Math.toRadians(90)) // place 1

                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(26, -36, Math.toRadians(90)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(45, -14, Math.toRadians(90)), Math.toRadians(0))
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(45, -48), Math.toRadians(90)) // push 2

                .splineToSplineHeading(new Pose2d(56, -14, Math.toRadians(90)), Math.toRadians(0))
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(56, -48), Math.toRadians(90)) // push 3

                .strafeToSplineHeading(new Vector2d(56, -40), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(64, -14, Math.toRadians(90)), Math.toRadians(0))
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(64, -48), Math.toRadians(90)) // push 4

                .splineToSplineHeading(new Pose2d(52, -47, Math.toRadians(90)), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(46.5, -62), Math.toRadians(-90)) // pick 2

                .strafeToLinearHeading(new Vector2d(3, -30), Math.toRadians(90)) // place 2

                .strafeToLinearHeading(new Vector2d(46.5, -62), Math.toRadians(90)) // pick 3

                .strafeToLinearHeading(new Vector2d(2, -30), Math.toRadians(90)) // place 3

                .strafeToLinearHeading(new Vector2d(46.5, -62), Math.toRadians(90)) // pick 4

                .strafeToLinearHeading(new Vector2d(1, -30), Math.toRadians(90)) // place 4

                .strafeToLinearHeading(new Vector2d(46.5, -62), Math.toRadians(90)) // pick 5

                .strafeToLinearHeading(new Vector2d(0, -30), Math.toRadians(90)) // place 5

                .strafeToLinearHeading(new Vector2d(32, -60), Math.toRadians(90)) // park


                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}