package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.opmode.PoseStorage;

@Config
@TeleOp(name = "Competition_2026", group = "drive")
public class CompetitionDrive extends LinearOpMode {

    private DcMotor intakeMotor;
    private DcMotorEx shoot_u, shoot_d, transfer_motor;
    private Servo s_transfer, s_block;
    private SampleMecanumDrive drive;

    // === PID Constants ===
    public static double P = 30, I = 2, D = 0, F = 13.5;
    public static double SHOOT_TARGET_RPM = 1400;

    // === PD Holding ===
    public static double HOLD_KP = 0.035, HOLD_KD = 0.14, HOLD_KH = 0.45;

    // === DRIVE ===
    public static double DRIVE_POWER = 0.8;
    public static double ROTATION_POWER = 0.8;

    // === SERVO POSITIONS ===
    public static double OPEN_POS = 0, CLOSE_POS = 0.06;
    public static double B_OPEN = 0.572, B_CLOSE = 0.52;

    public static boolean updatePID = false;

    private Pose2d targetPose = new Pose2d();
    private boolean holding = false;
    private boolean readyToShoot = false;
    private boolean lastGamepad1B = false;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        // 🔥 טעינת מיקום מהאוטונומי - חשוב לוודא שהאוטונומי שמר אותו ל-PoseStorage
        drive.setPoseEstimate(PoseStorage.currentPose);

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        shoot_u = hardwareMap.get(DcMotorEx.class, "shoot_u");
        shoot_d = hardwareMap.get(DcMotorEx.class, "shoot_d");
        s_transfer = hardwareMap.get(Servo.class, "s_transfer");
        s_block = hardwareMap.get(Servo.class, "s_block");
        transfer_motor = hardwareMap.get(DcMotorEx.class, "transfer_motor");


        shoot_u.setDirection(DcMotorSimple.Direction.FORWARD);
        shoot_d.setDirection(DcMotorSimple.Direction.REVERSE);
        s_transfer.setDirection(Servo.Direction.REVERSE);

        shoot_u.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot_d.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        updatePIDCoefficients();

        s_transfer.setPosition(OPEN_POS);
        s_block.setPosition(B_CLOSE);

        // === לולאת INIT ===
        // מציג את המיקום שנטען מהאוטונומי עוד לפני ה-Start
        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("=== COMPETITION READY ===");
            telemetry.addLine("Position Loaded from Autonomous:");
            displayPose();
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            Pose2d pose = drive.getPoseEstimate();
            Pose2d poseVelocity = drive.getPoseVelocity();
            if (poseVelocity == null) poseVelocity = new Pose2d();

            // איפוס זווית (Heading Reset)
            if (gamepad1.back) {
                drive.setPoseEstimate(new Pose2d(pose.getX(), pose.getY(), 0));
            }

            // === 1. INTAKE ===
            if (gamepad1.left_trigger > 0.1) {
                intakeMotor.setPower(0.8);
            } else if (gamepad1.right_trigger > 0.1) {
                intakeMotor.setPower(-0.8);
            } else {
                intakeMotor.setPower(0);
            }

            // === 2. SHOOTER TOGGLE (B) ===
            if (gamepad1.b && !lastGamepad1B) {
                readyToShoot = !readyToShoot;
            }
            lastGamepad1B = gamepad1.b;

            double targetVel = (SHOOT_TARGET_RPM * 28.0) / 60.0;
            if (readyToShoot) {
                shoot_u.setVelocity(targetVel);
                shoot_d.setVelocity(targetVel);
            } else {
                shoot_u.setVelocity(0);
                shoot_d.setVelocity(0);
            }

            // === 3. SHOOT ACTION (X) ===
            if (gamepad1.x) {
                shoot();
            }

            // === 4. PID UPDATE ===
            if (updatePID) {
                updatePIDCoefficients();
                updatePID = false;
            }

            // === 5. DRIVE & HOLDING ===
            if (gamepad1.a) {
                targetPose = new Pose2d(23.4, 22, Math.toRadians(45));
                holding = true;
            } else if (gamepad1.y) {
                targetPose = new Pose2d(47.1, 10.7, Math.toRadians(75));
                holding = true;
            } else if (Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.right_stick_x) > 0.1) {
                holding = false;
            }

            if (holding) {
                executePD(pose, poseVelocity);
            } else {
                double ly = -gamepad1.left_stick_y * DRIVE_POWER;
                double lx = -gamepad1.left_stick_x * DRIVE_POWER;
                double rx = -gamepad1.right_stick_x * ROTATION_POWER;

                double botHeading = pose.getHeading();
                double rotX = lx * Math.cos(-botHeading) - ly * Math.sin(-botHeading);
                double rotY = lx * Math.sin(-botHeading) + ly * Math.cos(-botHeading);

                drive.setWeightedDrivePower(new Pose2d(rotX, rotY, rx));
            }

            // === TELEMETRY UPDATE ===
            telemetry.addData("Status", holding ? "HOLDING TARGET" : "MANUAL DRIVE");
            telemetry.addData("Shooter", readyToShoot ? "READY (1400 RPM)" : "OFF");
            displayPose(); // קריאה לפונקציית הצגת המיקום
            telemetry.update();
        }
    }

    private void shoot() {
        s_block.setPosition(B_OPEN);
        sleep(100);
        intakeMotor.setPower(-1.0);
        sleep(1000);
        s_transfer.setPosition(OPEN_POS);
        sleep(300);
        s_transfer.setPosition(CLOSE_POS);
        intakeMotor.setPower(0);
        s_block.setPosition(B_CLOSE);
    }

    private void updatePIDCoefficients() {
        PIDFCoefficients coeff = new PIDFCoefficients(P, I, D, F);
        shoot_u.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeff);
        shoot_d.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeff);
    }

    private void executePD(Pose2d pose, Pose2d poseVelocity) {
        double xError = targetPose.getX() - pose.getX();
        double yError = targetPose.getY() - pose.getY();
        double hError = Angle.normDelta(targetPose.getHeading() - pose.getHeading());

        double xCmd = (xError * HOLD_KP) - (poseVelocity.getX() * HOLD_KD);
        double yCmd = (yError * HOLD_KP) - (poseVelocity.getY() * HOLD_KD);
        double hCmd = (hError * HOLD_KH);

        double rotX = xCmd * Math.cos(-pose.getHeading()) - yCmd * Math.sin(-pose.getHeading());
        double rotY = xCmd * Math.sin(-pose.getHeading()) + yCmd * Math.cos(-pose.getHeading());
        drive.setWeightedDrivePower(new Pose2d(rotX, rotY, hCmd));
    }

    // פונקציית עזר להדפסת המיקום בפורמט קריא
    private void displayPose() {
        Pose2d currentPose = drive.getPoseEstimate();
        telemetry.addData("Pose X", "%.2f", currentPose.getX());
        telemetry.addData("Pose Y", "%.2f", currentPose.getY());
        telemetry.addData("Pose Heading", "%.2f°", Math.toDegrees(currentPose.getHeading()));
    }
}
