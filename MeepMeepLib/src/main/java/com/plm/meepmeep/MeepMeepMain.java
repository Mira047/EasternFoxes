package com.plm.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import javafx.geometry.Pos;

public class MeepMeepMain {

    public static void main(String[] args){
        MeepMeep meepMeep = new MeepMeep(800);


        /*
        .lineTo(new Vector2d(35,-64))
        .waitSeconds(0.3)
        .lineTo(new Vector2d(10,-64))
        .waitSeconds(0.3)
         */

        double xPos = 12;
        double xPosIntake = 40;
        double xPosWarehouse = 25;
        double timeIntake = 1;
        double timeOuttake = 2;
        double timeDuck = 1;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(12,15)
                .setConstraints(45, 45  , Math.toRadians(270), Math.toRadians(270), 12)
                .followTrajectorySequence(drive ->
                            /*
                                drive.trajectorySequenceBuilder(new Pose2d(5, -64, Math.toRadians(0)))
                                        .lineTo(new Vector2d(xPosWarehouse,-64))
                                        .splineTo(new Vector2d(xPosIntake,-60),Math.toRadians(20))
                                        .waitSeconds(timeIntake)
                                        .lineToLinearHeading(new Pose2d(xPosWarehouse+10,-64,Math.toRadians(0)))
                                        //.lineToLinearHeading(new Pose2d(xPosWarehouse,-64,Math.toRadians(0)))
                                        .lineTo(new Vector2d(xPos,-64))
                            .build()
                        */
                        drive.trajectorySequenceBuilder(new Pose2d(5, -64, Math.toRadians(0)))


                                .splineToLinearHeading(new Pose2d(xPosIntake,-60,Math.toRadians(15)),Math.toRadians(25))
                                //.lineToLinearHeading(new Pose2d(xPosWarehouse,-64,Math.toRadians(0)))
                                //.lineToLinearHeading(new Pose2d(xPosIntake,-60,Math.toRadians(15)))
                                .waitSeconds(timeIntake)
                                .splineToLinearHeading(new Pose2d(xPos,-64,Math.toRadians(0)),Math.toRadians(20))
                                .waitSeconds(100)
                                .splineToLinearHeading(new Pose2d(xPosWarehouse,-64,Math.toRadians(0)),Math.toRadians(30))
                                //.lineToLinearHeading(new Pose2d(xPosWarehouse,-64,Math.toRadians(0)))
                                .lineTo(new Vector2d(xPos,-64))
                                .waitSeconds(timeOuttake)

                                .lineTo(new Vector2d(xPosIntake,-64))
                                .waitSeconds(timeIntake)
                                .lineTo(new Vector2d(xPos,-64))
                                .waitSeconds(timeOuttake)

                                .lineTo(new Vector2d(xPosIntake,-64))
                                .waitSeconds(timeIntake)
                                .lineTo(new Vector2d(xPos,-64))
                                .waitSeconds(timeOuttake)

                                .lineTo(new Vector2d(35,-64))

                                .build()


                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}