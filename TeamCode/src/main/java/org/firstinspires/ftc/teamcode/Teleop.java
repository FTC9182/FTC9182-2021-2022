package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Teleop", group = "teleop")
public class Teleop extends OpMode {
    private MecanumDrive _drive;
    private DcMotor _mc;
    private DcMotor _ms;

    public void init() {
        _drive = new MecanumDrive(hardwareMap);
        _mc  = hardwareMap.get(DcMotor.class, "motorcarousel");
//        motorconveyorbelt  = hwMap.get(DcMotor.class, "motorconveyorbelt");
        _ms  = hardwareMap.get(DcMotor.class, "motorstring");    }

    @Override
    public void start() {
    }

    public void loop() {

        _drive.setVelocity(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_button);

        _drive.update();

        _drive.displayTelemetry(telemetry);

        if (gamepad1.a) {
            _mc.setPower(.5);
        }
        else if (gamepad1.b) {
            _mc.setPower(-.5);
        }
        else {
            _mc.setPower(0);
        }

        if (gamepad1.dpad_left) {
            _ms.setPower(.5);
        } else if (gamepad1.dpad_right) {
            _ms.setPower(-.5);
        } else {
            _ms.setPower(0);
        }
        telemetry.update();
    }

    @Override
    public  void stop() {
    }
}
