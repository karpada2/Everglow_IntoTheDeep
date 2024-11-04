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
        MeepMeep meepMeep = new MeepMeep(800);



        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        Pose2d beginPose = new Pose2d(58, -63,  3 * (1./4)*Math.PI);
        TrajectoryActionBuilder start_2_box = myBot.getDrive().actionBuilder(beginPose)
                .waitSeconds(2)
                .strafeToSplineHeading(new Vector2d(58,-38),  3* (1./4)*Math.PI )
                // goes to right sample
                .waitSeconds(1) // grabs sample
                .setTangent(Math.PI)

                .splineToSplineHeading(new Pose2d(-56,-56,1.25*Math.PI), Math.PI)
                // strafe-turns and then splines to basket
                .waitSeconds(1) //elevator
                .strafeToSplineHeading(new Vector2d(-36,-34),0.75*Math.PI)
                // goes to right sample on the left
                .waitSeconds(1) //grabs sample
                .strafeToSplineHeading(new Vector2d(-56,-56),1.25*Math.PI)
                // goes to basket
                .waitSeconds(1) //elevator
                .strafeToSplineHeading(new Vector2d(-48, -34),0.75*Math.PI)
                // goes to middle sample on the left
                .waitSeconds(1) //grabs sample
                .strafeToSplineHeading(new Vector2d(-56,-56),1.25*Math.PI)
                // goes to basket
                .waitSeconds(1) //elevator
                .strafeToSplineHeading(new Vector2d(-57, -34),0.75*Math.PI)
                // goes to left sample on the left
                .waitSeconds(1) //grabs sample
                .strafeToSplineHeading(new Vector2d(-56,-56),1.25*Math.PI)
                // goes to basket
                .waitSeconds(1) //elevator
//                .strafeToSplineHeading(new Vector2d(-56,-36),Math.PI/2)
                // goes to sample pool
                .setTangent(Math.PI * 0.5)
                .splineToSplineHeading(new Pose2d(-26,0, 0),0)
                .waitSeconds(1) //grabs sample
                .strafeToSplineHeading(new Vector2d(-36,-10),-0.75*Math.PI)
                .splineTo(new Vector2d(-56,-56),-0.75*Math.PI)
                .waitSeconds(1) //elevator
                ;

//                .splineToConstantHeading(new Vector2d(0,-60), Math.PI);
//                .lineToYSplineHeading(33, Math.toRadians(0));
//                .lineToX(60);
//                .waitSeconds(2)
//                .setTangent(Math.toRadians(90))
//                .lineToY(48)
//                .setTangent(Math.toRadians(0))
//                .lineToX(32)
//                .strafeTo(new Vector2d(44.5, 30))
//                .turn(Math.toRadians(180))
//                .lineToX(47.5)
//                .waitSeconds(3);


//        Action start_2_box = myBot.getDrive().actionBuilder(beginPose)
//                .splineTo(new Vector2d(25, -11), Math.PI
//                )
//                .build();

//        Action start_2_box = myBot.getDrive().actionBuilder(beginPose)
//                .splineTo(new Vector2d(25, -11), Math.PI
//                )
//                .build();
        myBot.runAction(start_2_box.build());


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

