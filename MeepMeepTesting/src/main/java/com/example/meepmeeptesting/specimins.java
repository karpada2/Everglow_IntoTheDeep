package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class specimins {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(14,18.1)
                .build();

        double dropSpeciminY = -52;
        double firstSpeciminX = 50;
        double secondSpeciminX = 60;
        double PickSpeciminY = -12;

        Pose2d specimins_beginPose = new Pose2d(16, -62,   Math.PI/2);
        Pose2d specimins_basketPose = new Pose2d(0,-34,Math.PI/2);
        Pose2d specimins_endPose = new Pose2d(23,-10,0);
        Pose2d specimins_pickupPose = new Pose2d(48,-55,-Math.PI/2);



        // Start with the vertical elevator extended to the required height
        // Start with the horizontal elevator already extended as needed
        // The horizontal elevator could always be extended to the right amount
//        TrajectoryActionBuilder specimins_path_Preload = myBot.getDrive().actionBuilder(specimins_beginPose)
//                .setTangent((1)*Math.PI)
//                .splineToLinearHeading(new Pose2d(0,-34, Math.PI/2),Math.PI/2)
//                // Lower the vertical elevator
//                .waitSeconds(1)
//                .strafeToSplineHeading(specimins_dropPose.position, specimins_dropPose.heading)
//                // Claw drops sample (in case it didn't drop earlier)
//                .waitSeconds(3)
//                ;


        TrajectoryActionBuilder specimins_path= myBot.getDrive().actionBuilder(specimins_beginPose)
                .waitSeconds(2)
                // Goes to basket
                .strafeTo(specimins_basketPose.position)
                //Hangs
                .setTangent(-Math.PI/4)
//                .strafeToSplineHeading(specimins_dropPose.position, specimins_dropPose.heading)

                .splineToConstantHeading(new Vector2d(33,-33),Math.PI/2)
                .splineToSplineHeading(new Pose2d(firstSpeciminX,PickSpeciminY,Math.PI),0)
                .strafeToConstantHeading(new Vector2d(firstSpeciminX,dropSpeciminY))
                // Moves the second sample
                .splineToConstantHeading(new Vector2d(secondSpeciminX-4,PickSpeciminY),0)
                .waitSeconds(1)
//                .setTangent(-Math.PI/2)
                .strafeTo(new Vector2d(secondSpeciminX,dropSpeciminY))
                .setTangent(Math.PI/2)
//                .splineToSplineHeading(new Pose2d(specimins_pickupPose.position.x,specimins_pickupPose.position.y+10, -Math.PI/2),Math.PI)
                .splineToLinearHeading(new Pose2d(specimins_pickupPose.position,-Math.PI/2),-Math.PI/2)

                // Goes to the basket
                .setTangent((1) * Math.PI)
                .strafeToSplineHeading(specimins_basketPose.position, specimins_basketPose.heading)
                //Hangs Specimin

                .strafeToLinearHeading(specimins_pickupPose.position,-Math.PI/2)
                // Hangs Specimin
                .strafeToLinearHeading(specimins_basketPose.position,Math.PI/2)
                // Park
                .strafeToLinearHeading(new Vector2d(60,-60),Math.PI);

        myBot.runAction(specimins_path.build());




































        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
