package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(600);



        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        Pose2d left_beginPose = new Pose2d(-31.1, -61,   (1./2)*Math.PI);
        TrajectoryActionBuilder left_path = myBot.getDrive().actionBuilder(left_beginPose)
                .waitSeconds(2)
                .strafeToSplineHeading(new Vector2d(-51,-51),1.25*Math.PI)
                // goes to basket
                .waitSeconds(1) //puts sample in basket
                .strafeToSplineHeading(new Vector2d(-48,-38),0.5*Math.PI)
                // goes to right sample on the left
                .waitSeconds(1) //grabs sample remove
                //elevator
                .strafeToSplineHeading(new Vector2d(-51,-51),1.25*Math.PI)
                // goes to basket
                .strafeToSplineHeading(new Vector2d(-58, -38),0.5*Math.PI)
                // goes to middle sample on the left
                .waitSeconds(1) //grabs sample
                //elevator
                .strafeToSplineHeading(new Vector2d(-51,-51),1.25*Math.PI)
                // goes to basket
                .strafeToSplineHeading(new Vector2d(-57, -26),Math.PI)
                // goes to left sample on the left
                .waitSeconds(1) //grabs sample
                //elevator
                // goes to basket
                .strafeToSplineHeading(new Vector2d(-51,-51),1.25*Math.PI)
                // goes to sample pool
                .setTangent(Math.PI * 0.5)
                .splineToSplineHeading(new Pose2d(-34,0, 0),0)
                .waitSeconds(1) //grabs sample
                //elevator
                .setTangent(-(0.75)*Math.PI)
                .splineToSplineHeading(new Pose2d(-51,-51,1.25*Math.PI),1.25*Math.PI)
                .waitSeconds(1)
                .setTangent((0.25)*Math.PI)
                .strafeToLinearHeading(new Vector2d(-31.1, -61),Math.PI/2)
                ;


        myBot.runAction(left_path.build());

        Pose2d right_beginPose = new Pose2d(31.1, -61,   (1./2)*Math.PI);
        TrajectoryActionBuilder right_path = myBot.getDrive().actionBuilder(right_beginPose)
                .waitSeconds(10)
                .strafeToSplineHeading(new Vector2d(31.11,-61),0.5*Math.PI)
                ;


        myBot.runAction(left_path.build());
//        Pose2d beginPose2 = new Pose2d(20, -63,   (1./2)*Math.PI);
//        TrajectoryActionBuilder right_start_path = myBot.getDrive().actionBuilder(beginPose2)
//                .waitSeconds(2)
//                .strafeToSplineHeading(new Vector2d(36,-34),0.75*Math.PI - Math.PI /2)
//                // goes to right sample on the left
//                .waitSeconds(1) //grabs sample remove
//                //elevator
//                .strafeToSplineHeading(new Vector2d(52,-51),-Math.PI /2)
//                // goes to basket
//                .strafeToSplineHeading(new Vector2d(48, -34),0.75*Math.PI- Math.PI /2)
//                // goes to middle sample on the left
//                .waitSeconds(1) //grabs sample
//                //elevator
//                .strafeToSplineHeading(new Vector2d(54,-51),-Math.PI /2)
//                // goes to basket
//                .strafeToSplineHeading(new Vector2d(57, -34),0.75*Math.PI- Math.PI /2)
//                // goes to left sample on the left
//                .waitSeconds(1) //grabs sample
//                //elevator
//                // goes to basket
//                .strafeToSplineHeading(new Vector2d(56,-51),-Math.PI /2)
//                // goes to sample pool
//                .setTangent(Math.PI * 0.5)
//                .splineToSplineHeading(new Pose2d(26,0, Math.PI),Math.PI)
//                .waitSeconds(1) //grabs sample
//                //elevator
//                .setTangent(-(0.75)*Math.PI + Math.PI / 2)
//                .splineToSplineHeading(new Pose2d(58,-51,- Math.PI / 2),- Math.PI / 2)
//                ;
//        myBot.runAction(right_start_path.build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
//public class MeepMeepTesting {
//    public static void main(String[] args)
//    {
//        System.setProperty("sun.java2d.opengl", "true");
//        MeepMeep meepMeep = new MeepMeep(800);
//
//        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//                .build();
//

