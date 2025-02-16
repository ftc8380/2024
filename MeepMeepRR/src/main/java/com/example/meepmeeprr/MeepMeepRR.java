package com.example.meepmeeprr;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepRR {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // CONFIG VARIABLES (same as your FTC config)
        double startPoseX = 36;
        double startPoseY = 60;
        double startPoseHeadingDeg = -90;
        double basketX = 48;
        double basketY = 48;
        double basketSplineHeadingDeg = 45;
        double forwardDistance = 4.5;
        double backwardDistance = 5.0;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(
                                        new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseHeadingDeg)))
                                // 1) Spline to the basket position
                                .splineTo(new Vector2d(basketX, basketY),
                                        Math.toRadians(basketSplineHeadingDeg))
                                // 2) Drive forward (robot-relative)
                                .forward(forwardDistance)
                                // 3) Drive backward (robot-relative)
                                .back(backwardDistance)
                                .splineTo(new Vector2d(47, 45), Math.toRadians(90))
                                .splineTo(new Vector2d(basketX, basketY),
                                        Math.toRadians(basketSplineHeadingDeg))
                                // 2) Drive forward (robot-relative)
                                .forward(forwardDistance)
                                // 3) Drive backward (robot-relative)
                                .back(backwardDistance)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
