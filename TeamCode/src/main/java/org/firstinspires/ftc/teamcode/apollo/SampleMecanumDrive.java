package org.firstinspires.ftc.teamcode.apollo;

import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.Arrays;
import java.util.List;
import org.firstinspires.ftc.teamcode.apollo.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.apollo.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.apollo.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.apollo.util.LynxModuleUtil;

/**
 * SampleMecanumDrive - Basic mecanum drive system for the library.
 * This class handles motors and the IMU.
 */
public class SampleMecanumDrive {
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;
    private IMU imu;

    private Pose2d poseEstimate = new Pose2d();

    public SampleMecanumDrive(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, "frontleft ");
        leftRear = hardwareMap.get(DcMotorEx.class, "backleft");
        rightRear = hardwareMap.get(DcMotorEx.class, "backright");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontright");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
        imu.initialize(parameters);
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        double x = drivePower.getX();
        double y = drivePower.getY();
        double r = drivePower.getHeading();

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), 1);
        double frontLeftPower = (y + x + r) / denominator;
        double backLeftPower = (y - x + r) / denominator;
        double frontRightPower = (y - x - r) / denominator;
        double backRightPower = (y + x - r) / denominator;

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);
    }

    public double getRawExternalHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public Double getExternalHeadingVelocity() {
        return (double) imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
    }

    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    public void setPoseEstimate(Pose2d pose) {
        this.poseEstimate = pose;
    }
    
    public void update() {
        // בגרסה הזו של הספרייה, ה-update פשוט שומר על הפונקציונליות
    }
}
