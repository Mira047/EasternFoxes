package com.test.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    public static void main(String[] args){
        // Set window size to 70% of screen size
        int windowWidth = (int) (0.3 * java.awt.Toolkit.getDefaultToolkit().getScreenSize().getWidth());
        MeepMeep meepMeep = new MeepMeep(windowWidth);


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
                .setDimensions(13.7,13.7)
                .setConstraints(50, 50  , Math.toRadians(270), Math.toRadians(270), 13.7)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(34 , -63 , Math.toRadians(90)))
                        .waitSeconds(1)

                        .lineTo(new Vector2d(34 + 1,-63+30))
                        .splineToSplineHeading(new Pose2d(34 + 2,-63 + 53,Math.toRadians(90-40)),Math.toRadians(40))

                        .splineToSplineHeading(new Pose2d(34-10,-63+50,Math.toRadians(90 - 90)),Math.toRadians(180))
                        //.lineToLinearHeading(new Pose2d(34+3,-63+50,Math.toRadians(180)))
                        //.waitSeconds(10)
                        .splineTo(new Vector2d(34-24,-63+50),Math.toRadians(180))

                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}