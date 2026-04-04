package org.firstinspires.ftc.teamcode.apollo.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * IntakeSubsystem - Intake and transfer system. 
 * You can adjust motor power based on your needs!
 */
public class IntakeSubsystem {
    private DcMotor intakeMotor;
    private DcMotorEx transferMotor;

    public static double INTAKE_POWER = 0.8;
    public static double OUTTAKE_POWER = -0.8;
    public static double TRANSFER_POWER = 0.8;
    public static double TRANSFER_REVERSE = -0.3;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        transferMotor = hardwareMap.get(DcMotorEx.class, "transfer_motor");
    }

    public void startIntake() {
        intakeMotor.setPower(INTAKE_POWER);
        transferMotor.setPower(TRANSFER_REVERSE);
    }

    public void startOuttake() {
        intakeMotor.setPower(OUTTAKE_POWER);
        transferMotor.setPower(TRANSFER_POWER);
    }

    public void stop() {
        intakeMotor.setPower(0);
        transferMotor.setPower(0);
    }

    public void setTransferPower(double power) {
        transferMotor.setPower(power);
    }

    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }
}
