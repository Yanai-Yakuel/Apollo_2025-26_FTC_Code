package TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import mechanisims.mechanimDrive;

@TeleOp(name = "mechanimFieldOriented")
public class mechanimFieldOrientedOpMode extends OpMode {
    mechanimDrive drive = new mechanimDrive();

    public void init() {
        drive.init(hardwareMap);
    }

    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        drive.driveField(forward, strafe, rotate);
    }
}
