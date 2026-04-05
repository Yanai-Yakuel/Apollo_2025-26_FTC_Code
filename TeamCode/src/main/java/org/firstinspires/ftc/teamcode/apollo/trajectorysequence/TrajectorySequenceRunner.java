package org.firstinspires.ftc.teamcode.apollo.trajectorysequence;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import java.util.Collections;

/**
 * Runner class that handles execution of TrajectorySequences.
 */
@Config
public class TrajectorySequenceRunner {
    private TrajectoryFollower follower;

    public TrajectorySequenceRunner(TrajectoryFollower follower) {
        this.follower = follower;
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        // Simple implementation for now
        for (int i = 0; i < trajectorySequence.size(); i++) {
            Object obj = trajectorySequence.get(i);
            if (obj instanceof Trajectory) {
                follower.followTrajectory((Trajectory) obj);
            }
        }
    }

    public void update(Pose2d pose) {
        follower.update(pose);
    }

    public boolean isBusy() {
        return follower.isBusy();
    }
}
