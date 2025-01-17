package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

public class TrajectoryState implements State {
    private final MecanumDrive drive;
    private final Trajectory trajectory;

    public TrajectoryState(MecanumDrive drive, Trajectory trajectory) {
        this.drive = drive;
        this.trajectory = trajectory;
    }

    @Override
    public void init() {
        drive.followTrajectoryAsync(trajectory);
    }

    @Override
    public void run() {
        drive.update();
    }

    @Override
    public boolean isDone() {
        return !drive.isBusy();
    }
}