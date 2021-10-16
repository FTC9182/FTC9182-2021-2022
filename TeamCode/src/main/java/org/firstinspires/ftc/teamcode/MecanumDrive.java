package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.Collections;

public class MecanumDrive {

    private DcMotorEx[] motors;
    public static final String[] MOTOR_NAMES = {"motorLeftFront", "motorLeftBack", "motorRightBack", "motorRightFront"};
    private double[] powers;
    private double leftStickX;
    private double leftStickY;
    private double rightStickY;
    private double driveStickSpeed = .8;
    private double rotationStickSpeed = .8;

    private double currentPower = 1.4;

    public MecanumDrive(HardwareMap map) {
        powers = new double[4];
        motors = new DcMotorEx[4];

        for (int i = 0; i < 4; i ++) {
            DcMotorEx dcMotor = map.get(DcMotorEx.class, MOTOR_NAMES[i]);
            motors[i] = dcMotor;
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        motors[2].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[3].setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setVelocity(double leftStickX, double leftStickY, double rightStickY,
                            boolean leftStickPressed) {
        this.leftStickX = leftStickX;
        this.leftStickY = leftStickY;
        this.rightStickY = rightStickY;

        if (leftStickPressed) {
            rotationStickSpeed = .8;
            driveStickSpeed = .8;
        } else {
            driveStickSpeed = rotationStickSpeed = .4;
        }
    }

    public void update() {
        double x;
        double y;
        double rotation;
        double speed;

        x = Math.pow(-leftStickX, 3.0);
        y = Math.pow(leftStickY, 3.0);
        rotation = Math.pow(-rightStickY, 3.0) * rotationStickSpeed;
        speed = Math.min(1.0, Math.sqrt(x * x + y * y)) * driveStickSpeed;

        final double direction = Math.atan2(x, y);

        powers[0] = (speed * Math.sin(direction + Math.PI / 4.0) + rotation) * currentPower;
        powers[3] = (speed * Math.cos(direction + Math.PI / 4.0) - rotation) * currentPower;
        powers[1] = (speed * Math.cos(direction + Math.PI / 4.0) + rotation) * currentPower;
        powers[2] = (speed * Math.sin(direction + Math.PI / 4.0) - rotation) * currentPower;

        double max = Collections.max(Arrays.asList(1.0, Math.abs(powers[0]),
                Math.abs(powers[1]), Math.abs(powers[2]), Math.abs(powers[3])));

        for (int i = 0; i < 4; i++) {
            powers[i] /= max;
        }

        for (int i = 0; i < 4; i++) {
            motors[i].setPower(powers[i]);
        }
    }

    public void displayTelemetry(Telemetry telemetry) {
        for (int i = 0; i < 4; i++) {
            telemetry.addData(String.format("%s: position %d power %f", MOTOR_NAMES[i], motors[i].getCurrentPosition(), motors[i].getPower()), "");
        }

        telemetry.addData("x, y, r", "%f %f %f", leftStickX, leftStickY, rightStickY);
    }
}
