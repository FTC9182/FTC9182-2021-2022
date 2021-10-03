package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "Teleop", group = "Teleop code")
public class Teleop extends LinearOpMode
{
    private DcMotor motorleftfront;
    private DcMotor motorrightfront;
    private DcMotor motorleftback;
    private DcMotor motorrightback;
    @Override
    public void runOpMode() throws InterruptedException
    {
        motorleftfront = hardwareMap.dcMotor.get("motorleftfront")
        motorrightfront = hardwareMap.dcMotor.get("motorrightfront")
        motorleftback = hardwareMap.dcMotor.get("motorleftback")
        motorrightback = hardwareMap.dcMotor.get("motorrightback")

        motorleftfront.setDirection(DcMotor.Direction.Reverse);

        waitForStart();

        while(opModeIsActive())
        {
motorleftfront.setPower(-gamepad1.left_stick_y);
            motorrightfront.setPower(-gamepad1.right_stick_y);
            motorrightback.setPower(-gamepad1.right_stick_y);
            motorleftback.setPower(-gamepad1.left_stick_y);
            idle(
    }
}