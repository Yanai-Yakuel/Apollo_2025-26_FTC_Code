package org.firstinspires.ftc.teamcode.apollo.trajectorysequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import java.util.Collections;
import java.util.List;

/**
 * Interface representing a sequence of trajectories and markers.
 */
public class TrajectorySequence {
    private final List<Object> sequenceList;

    public TrajectorySequence(List<Object> sequenceList) {
        this.sequenceList = Collections.unmodifiableList(sequenceList);
    }

    public double duration() {
        double duration = 0.0;
        for (Object item : sequenceList) {
            if (item instanceof Trajectory) {
                duration += ((Trajectory) item).duration();
            }
            // Markers don't have duration
        }
        return duration;
    }

    public Pose2d start() {
        if (sequenceList.isEmpty()) return new Pose2d();
        Object first = sequenceList.get(0);
        if (first instanceof Trajectory) {
            return ((Trajectory) first).start();
        }
        return new Pose2d();
    }

    public Pose2d end() {
        if (sequenceList.isEmpty()) return new Pose2d();
        Object last = sequenceList.get(sequenceList.size() - 1);
        if (last instanceof Trajectory) {
            return ((Trajectory) last).end();
        }
        return new Pose2d();
    }

    public int size() {
        return sequenceList.size();
    }

    public Object get(int i) {
        return sequenceList.get(i);
    }
}
