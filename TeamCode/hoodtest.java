package teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoTest extends OpMode {

    Servo hood;
    boolean flag = true;
    boolean flag2 = true;
    double pos = 0.1;

    @Override
    public void init() {
        hood = hardwareMap.get(Servo.class, "s_hood");
    }

    @Override
    public void loop() {
        if (gamepad1.a && flag) {
            pos += 0.1;
            flag = false;
        } else if (!gamepad1.a && !flag) flag = true;
        if (gamepad1.b && flag2) {
            pos += 0.1;
            flag2 = false;
        } else if (!gamepad1.b && !flag2) flag2 = true;

        pos = Math.min(1.0, pos);
        hood.setPosition(pos);
    }
}package teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoTest extends OpMode {

    Servo hood;
    boolean flag = true;
    boolean flag2 = true;
    double pos = 0.1;

    @Override
    public void init() {
        hood = hardwareMap.get(Servo.class, "s_hood");
    }

    @Override
    public void loop() {
        if (gamepad1.a && flag) {
            pos += 0.1;
            flag = false;
        } else if (!gamepad1.a && !flag) flag = true;
        if (gamepad1.b && flag2) {
            pos += 0.1;
            flag2 = false;
        } else if (!gamepad1.b && !flag2) flag2 = true;

        pos = Math.min(1.0, pos);
        hood.setPosition(pos);
    }
}package teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoTest extends OpMode {

    Servo hood;
    boolean flag = true;
    boolean flag2 = true;
    double pos = 0.1;

    @Override
    public void init() {
        hood = hardwareMap.get(Servo.class, "s_hood");
    }

    @Override
    public void loop() {
        if (gamepad1.a && flag) {
            pos += 0.1;
            flag = false;
        } else if (!gamepad1.a && !flag) flag = true;
        if (gamepad1.b && flag2) {
            pos += 0.1;
            flag2 = false;
        } else if (!gamepad1.b && !flag2) flag2 = true;

        pos = Math.min(1.0, pos);
        hood.setPosition(pos);
    }
}
