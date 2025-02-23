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
                .setDimensions(16, 17)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 10)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-32, -62.5, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-52, -52), Math.toRadians(45)) // drop preload
                .strafeToLinearHeading(new Vector2d(-48, -40), Math.toRadians(90)) // pick 1
                .strafeToLinearHeading(new Vector2d(-52, -52), Math.toRadians(45)) // score 1
                .strafeToLinearHeading(new Vector2d(-58, -40), Math.toRadians(90)) // pick 2
                .strafeToLinearHeading(new Vector2d(-52, -52), Math.toRadians(45)) // score 2
                .strafeToLinearHeading(new Vector2d(-56, -25), Math.toRadians(180)) // pick 3
                .strafeToLinearHeading(new Vector2d(-50, -25), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-52, -52), Math.toRadians(45)) // score 3
                .strafeToLinearHeading(new Vector2d(-36, -10), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-24, -10), Math.toRadians(180)) // park
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}