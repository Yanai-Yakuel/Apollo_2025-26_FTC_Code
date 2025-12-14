package mechanisims;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class MechanumDrive {

    private DcMotor frontLeft, frontRight, backRight, backLeft;
    private IMU imu;

    public void init(HardwareMap hardwareMap) {
        // Motors
        frontLeft = hardwareMap.get(DcMotor.class,"frontLeft");
        frontRight = hardwareMap.get(DcMotor.class,"frontRight");
        backRight = hardwareMap.get(DcMotor.class,"backRight");
        backLeft = hardwareMap.get(DcMotor.class,"backLeft");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        );

        IMU.Parameters parameters = new IMU.Parameters(orientation);


        imu.initialize(parameters);
        imu.resetYaw();
    }

    public void drive(double forward, double strafe, double rotate) {
        double fl = forward + strafe + rotate;
        double bl = forward - strafe + rotate;
        double fr = forward - strafe - rotate;
        double br = forward + strafe - rotate;

        double maxPower = Math.max(Math.abs(fl),
                Math.max(Math.abs(bl), Math.max(Math.abs(fr), Math.abs(br))));

        if (maxPower > 1.0) {
            fl /= maxPower;
            bl /= maxPower;
            fr /= maxPower;
            br /= maxPower;
        }

        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        frontRight.setPower(fr);
        backRight.setPower(br);
    }

    public void driveFieldRelative(double forward, double strafe, double rotate) {
        if (imu == null) return;

        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        double yaw = angles.getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS);

        double temp = forward * Math.cos(-yaw) + strafe * Math.sin(-yaw);
        strafe = -forward * Math.sin(-yaw) + strafe * Math.cos(-yaw);
        forward = temp;

        drive(forward, strafe, rotate);
    }

    public void resetIMU() {
        if (imu != null) {
            imu.resetYaw();
        }
    }
    /**
     */
    public double getIMUAngles() {
        if (imu == null) return 0.0;
        return imu.getRobotYawPitchRollAngles().getYaw(
                org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES
        );
    }
}