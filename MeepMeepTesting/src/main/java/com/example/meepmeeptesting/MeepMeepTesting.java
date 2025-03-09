package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity sampleBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(16, 16)
                .setColorScheme(new ColorSchemeRedDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 10)
                .build();

        sampleBot.runAction(sampleBot.getDrive().actionBuilder(new Pose2d(-32, -62.5, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(45)) // drop preload
                .strafeToLinearHeading(new Vector2d(-47.5, -40.25), Math.toRadians(90)) // pick 1
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(45)) // score 1
                .strafeToLinearHeading(new Vector2d(-58, -41.5), Math.toRadians(90)) // pick 2
                .strafeToLinearHeading(new Vector2d(-53.5, -53.5), Math.toRadians(45)) // score 2
                .strafeToLinearHeading(new Vector2d(-53.5, -26.5), Math.toRadians(180)) // pick 3
                .strafeToLinearHeading(new Vector2d(-45, -25), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-53.5, -53.5), Math.toRadians(45)) // score 3
                .strafeToLinearHeading(new Vector2d(-36, -11.5), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-17, -11.5), Math.toRadians(180)) // park
                .build());

//        RoadRunnerBotEntity specimen5Bot = new DefaultBotBuilder(meepMeep)
//                .setDimensions(16, 16)
//                .setColorScheme(new ColorSchemeRedDark())
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 10)
//                .build();
//
//        specimen5Bot.runAction(specimen5Bot.getDrive().actionBuilder(new Pose2d(15, -62.5, Math.toRadians(90)))
//
//                .strafeToLinearHeading(new Vector2d(3, -28.75), Math.toRadians(90)) // place 1
//
//                .setTangent(Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(26, -36), Math.toRadians(0))
//                .splineToSplineHeading(new Pose2d(45, -14, Math.toRadians(180)), Math.toRadians(0))
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(45, -54, Math.toRadians(180)), Math.toRadians(-90)) // push 2
//
////                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(43, -30, Math.toRadians(180)), Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(56, -14, Math.toRadians(180)), Math.toRadians(0))
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(56, -54, Math.toRadians(180)), Math.toRadians(-90)) // push 3
//
////                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(54, -30, Math.toRadians(180)), Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(64.5, -14, Math.toRadians(180)), Math.toRadians(0))
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(64.5, -54, Math.toRadians(180)), Math.toRadians(-90)) // push 4
//
////                .setTangent(Math.toRadians(90))
//                .splineToSplineHeading(new Pose2d(52, -47, Math.toRadians(90)), Math.toRadians(180))
//                .splineToSplineHeading(new Pose2d(46.5, -62, Math.toRadians(90)), Math.toRadians(-90)) // pick 2
//
//                .strafeToLinearHeading(new Vector2d(1, -28.75), Math.toRadians(90)) // place 2
//
//                .strafeToLinearHeading(new Vector2d(46.5, -62), Math.toRadians(90)) // pick 3
//
//                .strafeToLinearHeading(new Vector2d(0.5, -28.75), Math.toRadians(90)) // place 3
//
//                .strafeToLinearHeading(new Vector2d(46.5, -62), Math.toRadians(90)) // pick 4
//
//                .strafeToLinearHeading(new Vector2d(0, -28.75), Math.toRadians(90)) // place 4
//
//                .strafeToLinearHeading(new Vector2d(46.5, -62), Math.toRadians(90)) // pick 5
//
//                .strafeToLinearHeading(new Vector2d(-0.5, -28.75), Math.toRadians(90)) // place 5
//
//                .strafeToLinearHeading(new Vector2d(32, -60), Math.toRadians(90)) // park
//
//
//                .build());
        
        RoadRunnerBotEntity specimen3Bot = new DefaultBotBuilder(meepMeep)
            .setDimensions(16, 16)
            .setColorScheme(new ColorSchemeRedDark())
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 10)
            .build();
        
        specimen3Bot.runAction(specimen3Bot.getDrive().actionBuilder(new Pose2d(15, -62.5, Math.toRadians(90)))
            
            .strafeToLinearHeading(new Vector2d(3, -28.625), Math.toRadians(90)) // place 1
            
            .setTangent(Math.toRadians(-90))
            .splineToConstantHeading(new Vector2d(26, -36), Math.toRadians(0))
            .splineToLinearHeading(new Pose2d(47, -14, Math.toRadians(180)), Math.toRadians(0))
            .setTangent(Math.toRadians(-90))
            .splineToLinearHeading(new Pose2d(47, -50, Math.toRadians(180)), Math.toRadians(-90)) // push 2

            .setTangent(Math.toRadians(90))
//            .splineToLinearHeading(new Pose2d(43, -30, Math.toRadians(180)), Math.toRadians(90))
            .splineToLinearHeading(new Pose2d(56, -14, Math.toRadians(180)), Math.toRadians(0))
            .setTangent(Math.toRadians(-90))
            .splineToLinearHeading(new Pose2d(56, -50, Math.toRadians(180)), Math.toRadians(-90)) // push 3

//                .setTangent(Math.toRadians(90))
//            .splineToSplineHeading(new Pose2d(52, -47, Math.toRadians(90)), Math.toRadians(180))
            .splineToLinearHeading(new Pose2d(46.5, -64.5, Math.toRadians(90)), Math.toRadians(-90)) // pick 2
            
            .strafeToLinearHeading(new Vector2d(0, -28.625), Math.toRadians(90)) // place 2
            
            .strafeToLinearHeading(new Vector2d(46.5, -64.5), Math.toRadians(90)) // pick 3
            
            .strafeToLinearHeading(new Vector2d(-0.5, -28.625), Math.toRadians(90)) // place 3
            
            .strafeToLinearHeading(new Vector2d(46.5, -64.5), Math.toRadians(90)) // pick 4
            
            .strafeToLinearHeading(new Vector2d(-1, -28.625), Math.toRadians(90)) // place 4
            
            .strafeToLinearHeading(new Vector2d(32, -60), Math.toRadians(90)) // park
            
            
            .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(sampleBot)
                .addEntity(specimen3Bot)
                .start();
    }
}