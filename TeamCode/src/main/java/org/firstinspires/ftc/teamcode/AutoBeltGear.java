package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//@Autonomous(name = "AutoBeltGear", group = "Test")
@TeleOp(name = "TestBeltGear", group = "Test")
public class AutoBeltGear extends LinearOpMode
{
    public DcMotor beltGearMotor = null;

    @Override
    public void runOpMode()
    {
        beltGearMotor = hardwareMap.get(DcMotor.class, "belt_gear_motor");
        waitForStart();

        while(opModeIsActive())
        {
            beltGearMotor.setPower(.4);
        }

    }
}
