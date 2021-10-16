package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp()
//@Disabled
public class WheelLocationClampTest extends OpMode {
    DcMotor _frontLeft;
    DcMotor _frontRight;
    DcMotor _backLeft;
    DcMotor _backRight;

    @Override
    public void init() {
        _frontLeft = hardwareMap.dcMotor.get("motorLeftFront");
        _frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _frontRight = hardwareMap.dcMotor.get("motorRightFront");
        _frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _backLeft = hardwareMap.dcMotor.get("motorLeftBack");
        _backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _backRight = hardwareMap.dcMotor.get("motorRightBack");
        _backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_left) {
            _frontLeft.setPower(.5);
        }

        if (!gamepad1.dpad_left) {
            _frontLeft.setPower(0);
        }

        if (gamepad1.dpad_up) {
            _frontRight.setPower(.5);
        }

        if (!gamepad1.dpad_up) {
            _frontRight.setPower(0);
        }

        if (gamepad1.dpad_right) {
            _backRight.setPower(.5);
        }

        if (!gamepad1.dpad_right) {
            _backRight.setPower(0);
        }

        if (gamepad1.dpad_down) {
            _backLeft.setPower(.5);
        }

        if (!gamepad1.dpad_down) {
            _backLeft.setPower(0);
        }

        telemetry.addData("fl", "%d", _frontLeft.getCurrentPosition());
        telemetry.addData("fr", "%d", _frontRight.getCurrentPosition());
        telemetry.addData("bl", "%d", _backLeft.getCurrentPosition());
        telemetry.addData("br", "%d", _backRight.getCurrentPosition());

        telemetry.update();
    }
}
