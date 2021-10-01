package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain {
    DcMotor frontRight = null;
    DcMotor frontLeft = null;
    DcMotor backLeft = null;
    DcMotor backRight = null;

    public DriveTrain(HardwareMap hardwareMap) {
        frontRight = hardwareMap.dcMotor.get("front_right");
        frontLeft = hardwareMap.dcMotor.get("front_left");
        backLeft = hardwareMap.dcMotor.get("back_left");
        backRight = hardwareMap.dcMotor.get("back_right");

        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    public void Move(double speedVari,double driveX, double driveY, double turnDegrees)
    {
        frontRight.setPower(speedVari*(-driveY-driveX-turnDegrees));
        frontLeft.setPower(speedVari*(-driveY+driveX+turnDegrees));
        backLeft.setPower(speedVari*(-driveY-driveX+turnDegrees));
        backRight.setPower(speedVari*(-driveY+driveX-turnDegrees));
    }

}
