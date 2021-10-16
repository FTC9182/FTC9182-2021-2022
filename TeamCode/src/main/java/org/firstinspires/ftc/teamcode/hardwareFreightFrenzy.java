/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class hardwareFreightFrenzy
{
    /* Public OpMode members. */
    public DcMotor motorLeftFront = null;
    public DcMotor motorRightFront = null;
    public DcMotor motorLeftBack = null;
    public DcMotor motorRightBack = null;
    public DcMotor motorcarousel = null;
    public DcMotor motorconveyorbelt = null;
    public DcMotor motorstring = null;
    public RevRoboticsCoreHexMotor motornoodles = null;

    public Servo    leftClaw    = null;
    public Servo    rightClaw   = null;

    public static final double MID_SERVO       =  0.5 ;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public hardwareFreightFrenzy(){

    }

    /*public void driveMec(double xValue, double yValue, double rxValue)
    {
        double denom = Math.max(Math.abs(yValue) + Math.abs(xValue) + Math.abs(rxValue),1);
        double frontLeftPower = (yValue + xValue + rxValue) / denom;
        double backLeftPower = (yValue - xValue + rxValue) / denom;
        double backRightPower = (yValue + xValue + rxValue) / denom;
        double frontRightPower = (yValue + xValue + rxValue) / denom;

        motorLeftFront.setPower(frontLeftPower);
        motorLeftBack.setPower(backLeftPower);
        motorRightBack.setPower(backRightPower);
        motorRightFront.setPower(frontRightPower);
    }
    */

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        motorLeftFront  = hwMap.get(DcMotor.class, "motorLeftFront");
        motorLeftBack   = hwMap.get(DcMotor.class, "motorLeftBack");
        motorRightFront = hwMap.get(DcMotor.class, "motorRightFront");
        motorRightBack  = hwMap.get(DcMotor.class, "motorRightBack");
        /* Define and initialize other motors.
        motorcarousel  = hwMap.get(DcMotor.class, "motorcarousel");
        motorconveyorbelt  = hwMap.get(DcMotor.class, "motorconveyorbelt");
        motorstring  = hwMap.get(DcMotor.class, "motorstring");
        */
        //motornoodles  = hwMap.get(DcMotor.class, "");

        // set defaults
        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        // motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        motorRightFront.setDirection(DcMotor.Direction.FORWARD); //messed up????
        motorRightBack.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        motorLeftFront.setPower(0);
        motorRightFront.setPower(0);
        motorLeftBack.setPower(0);
        motorRightBack.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }
 }

