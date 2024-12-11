package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(24, -60, Math.toRadians(-90)))
                        .strafeTo(new Vector2d(5, -36))
                        .back(3)
                        .forward(10)

                        /** TODO: Agregar parte de la garra y colgar **/

                        .splineToConstantHeading(new Vector2d(36, -50),Math.toRadians(-90))

                        .strafeTo(new Vector2d(36, -40))
                        .back(32)
                        .strafeLeft(10)
                        .lineTo(new Vector2d(50,-50))
                        .lineTo(new Vector2d(50, -38))
                        .strafeTo(new Vector2d(58, -50))
                        //.lineTo(new Vector2d(58,-50))
                        .lineToLinearHeading(new Pose2d(47,-50, Math.toRadians(90)))
                        //.strafeTo(new Vector2d(47, -50))
                        /**de aquí a colgarlo de nuevo**/
                        .strafeTo(new Vector2d(5,-36))

                        //.lineTo(new Vector2d(58,-8))
                        //.lineToLinearHeading(new Pose2d(24, -8, Math.toRadians(0)))

                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}