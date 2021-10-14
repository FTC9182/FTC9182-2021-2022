package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(name = "Freight Teleop", group = "9182")
public class Teleop extends LinearOpMode
{

    /* Declare OpMode members. */
    hardwareFreightFrenzy robot   = new hardwareFreightFrenzy();            // Use our hardware profile
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor motorleftfront;
    private DcMotor motorrightfront;
    private DcMotor motorleftback;
    private DcMotor motorrightback;


    @Override
    public void runOpMode() throws InterruptedException
    {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        //motorleftfront = hardwareMap.dcMotor.get("motorleftfront");
        //motorrightfront = hardwareMap.dcMotor.get("motorrightfront");
        //motorleftback = hardwareMap.dcMotor.get("motorleftback");
        //motorrightback = hardwareMap.dcMotor.get("motorrightback");

        // We show the log in oldest-to-newest order, as that's better for poetry
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        // We can control the number of lines shown in the log
        telemetry.log().setCapacity(6);

        waitForStart();

        while(opModeIsActive()) {
            motorleftfront.setPower(gamepad1.left_stick_y);
            motorrightfront.setPower(-gamepad1.left_stick_y);
            motorrightback.setPower(-gamepad1.left_stick_y);
            motorleftback.setPower(gamepad1.left_stick_y);
            motorleftfront.setPower(-gamepad1.left_stick_x);
            motorrightfront.setPower(-gamepad1.left_stick_x);
            motorrightback.setPower(-gamepad1.left_stick_x);
            motorleftback.setPower(-gamepad1.left_stick_x);
            idle();

            // Show joystick information as some other illustrative data
            telemetry.addLine("left joystick | ")
                    .addData("x", gamepad1.left_stick_x)
                    .addData("y", gamepad1.left_stick_y);
            telemetry.addLine("right joystick | ")
                    .addData("x", gamepad1.right_stick_x)
                    .addData("y", gamepad1.right_stick_y);

            // Update the screen
            telemetry.update();
        }
    }
}
