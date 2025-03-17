package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
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
        double firstSpeciminX = 45;
        double secondSpeciminX = 48;
        double thirdSpeciminX = 60;
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
                //starts at hanging height
                .waitSeconds(2)
                // Goes to basket
                .strafeTo(specimins_basketPose.position)
                //* Hangs
                // Pushes Both Specimins into place
                .setTangent(-Math.PI/4)
                .splineToConstantHeading(new Vector2d(33,-30),Math.PI/2)
                .splineToSplineHeading(new Pose2d(firstSpeciminX-10,PickSpeciminY,Math.PI),0)
                .splineToConstantHeading(new Vector2d(firstSpeciminX,dropSpeciminY),-Math.PI/2)
                // Moves to the second specimin
                .splineToConstantHeading(new Vector2d(secondSpeciminX,PickSpeciminY),0)
                // Waits for human player
                // Pushes the second specimin
                .splineToConstantHeading(new Vector2d(55,-45),Math.PI/1.6)
                .splineToSplineHeading(specimins_pickupPose,-Math.PI/2)

//                .splineToConstantHeading(new Vector2d(thirdSpeciminX,PickSpeciminY),0)
                // Waits for human player
                // Pushes the second specimin
//                .splineToConstantHeading(new Vector2d(thirdSpeciminX,dropSpeciminY),-Math.PI/2)
                //* Raises Elevator to picking up height
                // Goes to pickup first specimin
//                .setTangent(Math.PI/2)
//                .splineToLinearHeading(new Pose2d(specimins_pickupPose.position,-Math.PI/2),-Math.PI/2)
                //* Lowers elevator to pickup
                // Goes to basket
                // *Raises elevator for hang

                .strafeToSplineHeading(new Vector2d(0,-54), specimins_basketPose.heading)
                .strafeTo(specimins_basketPose.position, new TranslationalVelConstraint(150))
                //* Hangs Specimen sequentially

                //* Lowers Elevator to picking up height
                // Goes to pickup second specimen
                .strafeToLinearHeading(specimins_pickupPose.position,-Math.PI/2)
                .strafeToSplineHeading(new Vector2d(0,-54), specimins_basketPose.heading)
                .strafeTo(specimins_basketPose.position, new TranslationalVelConstraint(150))
                //* Picks up second specimen
                // Goes to hang Specimen
                .strafeToLinearHeading(specimins_pickupPose.position,-Math.PI/2)
                .strafeToSplineHeading(new Vector2d(0,-54), specimins_basketPose.heading)
                .strafeTo(specimins_basketPose.position, new TranslationalVelConstraint(150))
                //* Hangs
                //*Lowers elevator
                // Park
                .strafeToLinearHeading(new Vector2d(60,-60),Math.PI);

//        myBot.runAction(specimins_path.build());

        TrajectoryActionBuilder OnePlusOnespecimins_path= myBot.getDrive().actionBuilder(specimins_beginPose)
                //starts at hanging height
                .waitSeconds(2)
                // Goes to basket
                .strafeTo(specimins_basketPose.position)
                //* Hangs
                // Pushes Both Specimins into place

                .strafeToSplineHeading(specimins_pickupPose.position,-Math.PI/2)
                //* Lowers elevator to pickup
                // Goes to basket
                // *Raises elevator for hang

                .strafeToSplineHeading(specimins_basketPose.position,Math.PI/2)

                .setTangent(-Math.PI/4)
                .splineToConstantHeading(new Vector2d(firstSpeciminX-2,PickSpeciminY-30), Math.PI/2)


                .strafeToLinearHeading(new Vector2d(60,-60),Math.PI);

        myBot.runAction(specimins_path.build());



































        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
