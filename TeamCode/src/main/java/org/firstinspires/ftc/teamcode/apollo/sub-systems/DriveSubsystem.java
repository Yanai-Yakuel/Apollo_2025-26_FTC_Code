package org.firstinspires.ftc.teamcode.apollo.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.apollo.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.apollo.TwoWheelTrackingLocalizer;

/**
 * DriveSubsystem - Advanced drive system including Limelight-based auto-alignment.
 */
public class DriveSubsystem {
    public SampleMecanumDrive drive;
    private Limelight3A limelight;
    private TwoWheelTrackingLocalizer localizer;

    // PID alignment constants - can be adjusted!
    public static double HOLD_KP = 0.045;
    public static double HOLD_KI = 0.003;
    public static double HOLD_KD = 0.18;
    public static double HOLD_KH = 0.9;
    
    private double xIntegral = 0, yIntegral = 0;
    public static final double M_TO_IN = 39.37;

    public DriveSubsystem(HardwareMap hardwareMap) {
        drive = new SampleMecanumDrive(hardwareMap);
        localizer = new TwoWheelTrackingLocalizer(hardwareMap, drive);

        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            if (limelight != null) {
                limelight.pipelineSwitch(0);
                limelight.start();
            }
        } catch (Exception e) {
            limelight = null;
        }
    }

    public void update() {
        drive.update();
        // עדכון מיקום
        LLResult result = (limelight != null) ? limelight.getLatestResult() : null;
        if (result != null && result.isValid()) {
            updateBotPose(result);
        }
    }

    private void updateBotPose(LLResult result) {
        Pose3D botPose = result.getBotpose();
        if (botPose != null && botPose.getPosition() != null) {
            double x = botPose.getPosition().x * M_TO_IN;
            double y = botPose.getPosition().y * M_TO_IN;
            double yaw = Math.toRadians(botPose.getOrientation().getYaw());
            drive.setPoseEstimate(new Pose2d(x, y, yaw));
        }
    }

    public void alignToTarget(Pose2d target) {
        Pose2d currentPose = drive.getPoseEstimate();
        
        double xErr = target.getX() - currentPose.getX();
        double yErr = target.getY() - currentPose.getY();
        double hErr = Angle.normDelta(target.getHeading() - currentPose.getHeading());

        double distance = Math.hypot(xErr, yErr);

        if (distance < 0.5 && Math.abs(hErr) < Math.toRadians(2)) {
            drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
            xIntegral = 0; yIntegral = 0;
            return;
        }

        xIntegral = Math.max(-10, Math.min(10, xIntegral + xErr * 0.02));
        yIntegral = Math.max(-10, Math.min(10, yIntegral + yErr * 0.02));

        double xCmd = xErr * HOLD_KP + xIntegral * HOLD_KI;
        double yCmd = yErr * HOLD_KP + yIntegral * HOLD_KI;

        double maxSpeed = Math.min(0.7, distance * 0.1);
        double heading = currentPose.getHeading();
        double rotX = xCmd * Math.cos(-heading) - yCmd * Math.sin(-heading);
        double rotY = xCmd * Math.sin(-heading) + yCmd * Math.cos(-heading);

        rotX = Math.max(-maxSpeed, Math.min(maxSpeed, rotX));
        rotY = Math.max(-maxSpeed, Math.min(maxSpeed, rotY));

        double rotPower = Math.max(-0.5, Math.min(0.5, hErr * HOLD_KH));
        drive.setWeightedDrivePower(new Pose2d(rotX, rotY, rotPower));
    }

    public void stopLimelight() {
        if (limelight != null) limelight.stop();
    }
}
