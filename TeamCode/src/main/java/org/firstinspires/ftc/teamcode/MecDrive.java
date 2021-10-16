package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecDrive
{
    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;

    public void driveMec(double xValue, double yValue, double rxValue)
    {
       double denom = Math.max(Math.abs(yValue) + Math.abs(xValue) + Math.abs(rxValue),1);
       double frontLeftPower = (yValue + xValue + rxValue) / denom;
       double backLeftPower = (yValue - xValue + rxValue) / denom;
       double backRightPower = (yValue + xValue + rxValue) / denom;
       double frontRightPower = (yValue + xValue + rxValue) / denom;

       motorFrontLeft.setPower(frontLeftPower);
       motorBackLeft.setPower(backLeftPower);
       motorBackRight.setPower(backRightPower);
       motorFrontRight.setPower(frontRightPower);
    }
}
