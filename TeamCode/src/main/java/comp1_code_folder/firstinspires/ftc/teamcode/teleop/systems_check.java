package comp1_code_folder.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "systems_check", group = "test")
public class systems_check extends LinearOpMode {

    // Hardware
    private DcMotor intakeMotor, transferMotor;
    private DcMotorEx shoot_u, shoot_d;
    private Servo s_transfer;

    // Servo
    private double close_pos = 0.4;
    private final double open_pos = 0.92;

    // Motor / Encoder settings
    private final double MOTOR_TICKS_PER_REV = 28.0;
    private final double GEAR_RATIO = 1.5; // Motor:Wheel = 1:1.5

    // RPM variables
    private double shoot_u_motor_rpm = 0.0;
    private double shoot_d_motor_rpm = 0.0;
    private double shoot_u_wheel_rpm = 0.0;
    private double shoot_d_wheel_rpm = 0.0;

    // Additional variables
    private boolean lastUp = false, lastDown = false;
    private boolean lastB = false;
    private boolean shootingEnabled = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // Hardware mapping
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        transferMotor = hardwareMap.get(DcMotor.class, "transfer");
        shoot_u = hardwareMap.get(DcMotorEx.class, "shoot_u");
        shoot_d = hardwareMap.get(DcMotorEx.class, "shoot_d");
        s_transfer = hardwareMap.get(Servo.class, "s_transfer");

        // Reset encoders
        shoot_u.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot_d.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shoot_u.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoot_d.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shoot_u.setDirection(DcMotor.Direction.REVERSE);
        shoot_d.setDirection(DcMotor.Direction.FORWARD);

        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("🚀 SYSTEMS CHECK + RPM ✓");
        telemetry.addLine("B=Toggle Shoot | Y=Servo | A=Intake | X=Transfer");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            // Servo control
            if (gamepad1.dpad_up && !lastUp) close_pos += 0.01;
            if (gamepad1.dpad_down && !lastDown) close_pos -= 0.01;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            close_pos = Math.max(0.0, Math.min(1.0, close_pos));

            // Toggle shooting
            if (gamepad1.b && !lastB) shootingEnabled = !shootingEnabled;
            lastB = gamepad1.b;

            // Motor control
            intakeMotor.setPower(gamepad1.a ? -0.8 : 0);
            transferMotor.setPower(gamepad1.x ? 0.8 : 0);

            double shootPower = shootingEnabled ? 0.9 : 0.0;
            shoot_u.setPower(shootPower);
            shoot_d.setPower(shootPower);

            s_transfer.setPosition(gamepad1.y ? close_pos : open_pos);

            // Telemetry
            telemetry.clearAll();
            telemetry.addLine("=== SYSTEMS CHECK 2026 ===");
            telemetry.addData("Servo Close", "%.3f", close_pos);
            telemetry.addData("Shooting", shootingEnabled ? "ON" : "OFF");
            telemetry.addData("Servo Pos", gamepad1.y ? "CLOSED" : "OPEN");
            telemetry.addLine("");

            telemetry.addData("Shoot_U Motor RPM", "%.0f", shoot_u_motor_rpm);
            telemetry.addData("Shoot_D Motor RPM", "%.0f", shoot_d_motor_rpm);
            telemetry.addData("Shoot_U Wheel RPM", "%.0f", shoot_u_wheel_rpm);
            telemetry.addData("Shoot_D Wheel RPM", "%.0f", shoot_d_wheel_rpm);
            telemetry.addData("Target Wheel RPM", "4000");
            telemetry.addLine("");

            telemetry.addData("Intake", gamepad1.a ? "ON" : "OFF");
            telemetry.addData("Transfer", gamepad1.x ? "ON" : "OFF");
            telemetry.update();
        }
    }
}
