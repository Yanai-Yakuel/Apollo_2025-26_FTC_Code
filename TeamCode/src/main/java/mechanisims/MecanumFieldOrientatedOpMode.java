package mechanisims;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "FieldOriented_Telemetry", group="Examples")
public class MecanumFieldOrientatedOpMode extends OpMode {

    MechanumDrive drive = new MechanumDrive();

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing Drive System...");
        telemetry.update();

        drive.init(hardwareMap);

        telemetry.addData("Status", "Initialization Complete! Press Start.");
        telemetry.addData("Controls", "A button to reset Field Orientation");
        telemetry.update();
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double strafe  = gamepad1.left_stick_x;
        double rotate  = gamepad1.right_stick_x;

        // איפוס Field-Centric
        if (gamepad1.a) {
            drive.resetIMU();
            telemetry.addData("IMU Reset", "Yaw set to 0.0°");
        }

        drive.driveFieldRelative(forward, strafe, rotate);

        double yawAngle = drive.getIMUAngles();

        telemetry.addData("Yaw (Robot Direction)", "%.2f°", yawAngle);

        telemetry.addData("Input Forward", "%.2f", forward);
        telemetry.addData("Input Strafe", "%.2f", strafe);
        telemetry.addData("Input Rotate", "%.2f", rotate);

        telemetry.update();
    }
}