package comp2_code_folder.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;

/**
 * Simple static field that allows us to keep track of the robot's pose between
 * autonomous and teleop without losing the relative coordinate system.
 */
public class PoseStorage {
    public static Pose2d currentPose = new Pose2d();
}
