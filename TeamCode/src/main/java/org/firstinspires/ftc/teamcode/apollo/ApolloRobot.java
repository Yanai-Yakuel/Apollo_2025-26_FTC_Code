package org.firstinspires.ftc.teamcode.apollo;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.apollo.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.apollo.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.apollo.subsystems.ShooterSubsystem;

/**
 * ApolloRobot - The main class for your robot!
 * All systems are centralized here for easy access.
 */
public class ApolloRobot {
    public DriveSubsystem drive;
    public ShooterSubsystem shooter;
    public IntakeSubsystem intake;

    public ApolloRobot(HardwareMap hardwareMap) {
        drive = new DriveSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
    }

    public void update() {
        drive.update();
    }

    public void stop() {
        drive.stopLimelight();
    }
}
