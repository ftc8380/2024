package com.example.meepmeeprr;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepRR {
    public static void main(String[] args) {
        // Create MeepMeep instance with an 800x800 pixel window.
        MeepMeep meepMeep = new MeepMeep(800);

        // Define the starting pose.
        // (24, -63.3125) with a heading of 90° (i.e. Math.toRadians(90))
        Pose2d startPose = new Pose2d(24, -63.3125, Math.toRadians(90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set robot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width.
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                // 1) Spline to (0, -32) with heading 90°
                                .splineTo(new Vector2d(0, -32), Math.toRadians(90))
                                // 2) Forward 4 units
                                .forward(4)
                                // 3) Back 5 units then strafe to (36, -36)
                                .back(15)
                                .strafeTo(new Vector2d(36, -36))
                                // 4) Spline to (36, -8.7) with heading 0°
                                .strafeTo(new Vector2d(36, -8.7))
                                // 5) Strafe to (42, -8.7)
                                .strafeTo(new Vector2d(42, -8.7))
                                .turn(Math.toRadians(90))
                                // 6) Line to (42, -60)
                                .lineTo(new Vector2d(42, -60))
                                // 7) Line to (42, -40)
                                .lineTo(new Vector2d(42, -40))
                                // 8) Turn by -90° (clockwise)
                                .turn(Math.toRadians(90))
                                // 9) Line to (42, -56)
                                .lineTo(new Vector2d(42, -56))
                                // 10) Line to (42, 63.2)
                                .lineTo(new Vector2d(42, -63.2))
                                // 11) Back 5 units then spline to (3, -32) with heading 90°
                                .back(5)
                                .strafeTo(new Vector2d(3, -40))
                                // 12) Forward 4 units (trajectoryTwo repeated)
                                .turn(Math.toRadians(180))
                                .forward(15)
                                .back(15)
                                // 13) Strafe to (36, -60) to park
                                .strafeTo(new Vector2d(36, -60))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
