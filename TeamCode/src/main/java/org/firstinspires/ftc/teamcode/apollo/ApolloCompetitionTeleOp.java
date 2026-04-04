package org.firstinspires.ftc.teamcode.apollo;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.apollo.subsystems.ShooterSubsystem;

/**
 * ApolloCompetitionTeleOp - OpMode using the new library.
 * The code here is much shorter and cleaner!
 */
@TeleOp(name = "Apollo Competition TeleOp", group = "apollo")
public class ApolloCompetitionTeleOp extends LinearOpMode {
    private ApolloRobot robot;
    
    // Target position for alignment
    public static Pose2d TARGET_POSE_RED = new Pose2d(-26.7, 24.0175, Math.toRadians(135));

    private boolean readyToShoot = false;
    private boolean lastGamepad1B = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new ApolloRobot(hardwareMap);
        
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            robot.update();
            Pose2d currentPose = robot.drive.drive.getPoseEstimate();

            // Y button - auto align to target
            if (gamepad1.y) {
                robot.drive.alignToTarget(TARGET_POSE_RED);
            } else {
                // Manual stick drive
                double inputY = -gamepad1.left_stick_y * 0.9;
                double inputX = gamepad1.left_stick_x * 0.9;
                double rx = -gamepad1.right_stick_x * 0.9;

                double botHeading = robot.drive.drive.getRawExternalHeading();
                double rotX = inputX * Math.cos(-botHeading) - inputY * Math.sin(-botHeading);
                double rotY = inputX * Math.sin(-botHeading) + inputY * Math.cos(-botHeading);

                robot.drive.drive.setWeightedDrivePower(new Pose2d(rotX, rotY, rx));
            }

            // Intake
            if (gamepad1.left_trigger > 0.1) {
                robot.intake.startIntake();
            } else if (gamepad1.right_trigger > 0.1) {
                robot.intake.startOuttake();
            } else if (robot.shooter.getCurrentState() == ShooterSubsystem.ShootState.IDLE) {
                robot.intake.stop();
            }

            // Shooter
            if (gamepad1.b && !lastGamepad1B) {
                readyToShoot = !readyToShoot;
            }
            lastGamepad1B = gamepad1.b;

            robot.shooter.setShooterPower(readyToShoot);
            robot.shooter.update(gamepad1.dpad_up, robot.intake);

            // Simple telemetry
            telemetry.addData("Status", "Apollo Core Running");
            telemetry.addData("Shooter Ready", robot.shooter.isShooterReady());
            telemetry.addData("Pose", currentPose.toString());
            telemetry.update();
        }

        robot.stop();
    }
}
