package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Field Centric Drive")
public class FieldCentricDrive extends OpMode {

    private GoBildaPinpointDriver odo;
    private DcMotor backLeft, backRight, frontLeft, frontRight;

    @Override
    public void init() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        // כיווני מנועים
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // הגדרות odometry
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        for (DcMotor motor : new DcMotor[]{backLeft, backRight, frontLeft, frontRight}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        odo.resetPosAndIMU();
        Pose2D startingPosition = new Pose2D(DistanceUnit.MM, -923.925, 1601.47, AngleUnit.RADIANS, 0);
        odo.setPosition(startingPosition);

        telemetry.addData("status", "initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        // Deadzone
        if (Math.abs(forward) < 0.05) forward = 0;
        if (Math.abs(strafe) < 0.05) strafe = 0;
        if (Math.abs(rotate) < 0.05) rotate = 0;

        Pose2D pos = odo.getPosition();
        double heading = pos.getHeading(AngleUnit.RADIANS);

        // Field-centric transformation (תיקון החישוב)
        double cosAngle = Math.cos(Math.PI/2 - heading);
        double sinAngle = Math.sin(Math.PI/2 - heading);

        double globalForward = forward * cosAngle + strafe * sinAngle;
        double globalStrafe = -forward * sinAngle + strafe * cosAngle;

        // חישוב מהירויות גלגלים (סטנדרטי ל-mecanum)
        double frontLeftPower = globalForward + globalStrafe + rotate;
        double frontRightPower = globalForward - globalStrafe - rotate;
        double backLeftPower = globalForward - globalStrafe + rotate;
        double backRightPower = globalForward + globalStrafe - rotate;

        // נורמליזציה
        double max = Math.max(Math.abs(frontLeftPower), Math.max(
                Math.abs(frontRightPower), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
        ));
        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // הגדרת כוחות (תיקון הסדר!)
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        // Telemetry משופר
        telemetry.addData("X (mm)", String.format("%.1f", pos.getX(DistanceUnit.MM)));
        telemetry.addData("Y (mm)", String.format("%.1f", pos.getY(DistanceUnit.MM)));
        telemetry.addData("Heading (deg)", String.format("%.1f", Math.toDegrees(heading)));
        telemetry.addData("Powers", String.format("FL:%.2f FR:%.2f BL:%.2f BR:%.2f",
                frontLeftPower, frontRightPower, backLeftPower, backRightPower));
        telemetry.update();
    }
}
