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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;


@Autonomous(name="Left Auto Freight Frenzy", group="9182")
//@Disabled
public class LeftAutoFreightFrenzy extends LinearOpMode {

    private DcMotor motorleftfront;
    private DcMotor motorrightfront;
    private DcMotor motorleftback;
    private DcMotor motorrightback;

    /* Declare OpMode members. */
    hardwareFreightFrenzy   robot   = new hardwareFreightFrenzy();            // Use our hardware profile
    ModernRoboticsI2cGyro gyro    = null;                    // Additional Gyro device 
 
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable
    
    static int              marker_Location          = 1;        // Default guess


    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        
        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData("Status", "Calibrating Gyro");    //
        telemetry.update();

        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && gyro.isCalibrating())  {
            sleep(50);
            idle();
        }

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        
        robot.motorleftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorrightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorleftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorrightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorleftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorrightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorleftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorrightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Ready...",  "Starting at %7d :%7d :%7d :%7d",
                          robot.motorleftfront.getCurrentPosition(),
                          robot.motorrightfront.getCurrentPosition(),
                          robot.motorleftback.getCurrentPosition(),
                          robot.motorrightback.getCurrentPosition()
                        );
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        //waitForStart();

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
            telemetry.update();
        }

        gyro.resetZAxisIntegrator();
        

        // ------------------ Main code here, map out path -----------------
        // Note: Reverse movement is obtained by setting a negative distance (not speed)


        // --- use camera to get location (1,2,3) ---
        marker_Location = 1;


        // --- move to drop bin ---



        // --- levels determined by marker location ---
        if (marker_Location == 1) {
            // drop to lowest level
            gyroDrive(DRIVE_SPEED, 48.0, 0.0);    // Drive FWD 48 inches
            gyroTurn( TURN_SPEED, -45.0);         // Turn  CCW to -45 Degrees
            gyroHold( TURN_SPEED, -45.0, 0.5);    // Hold -45 Deg heading for a 1/2 second
            gyroDrive(DRIVE_SPEED, 12.0, -45.0);  // Drive FWD 12 inches at 45 degrees
            gyroTurn( TURN_SPEED,  45.0);         // Turn  CW  to  45 Degrees
            gyroHold( TURN_SPEED,  45.0, 0.5);    // Hold  45 Deg heading for a 1/2 second
            gyroTurn( TURN_SPEED,   0.0);         // Turn  CW  to   0 Degrees
            gyroHold( TURN_SPEED,   0.0, 1.0);    // Hold  0 Deg heading for a 1 second
            gyroDrive(DRIVE_SPEED,-48.0, 0.0);    // Drive REV 48 inches
        }
        else if (marker_Location == 2) {
            // drop to second level

        }
        else {
            // go to third level

        }


        //encoderDrive(DRIVE_SPEED,  48,  48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        //encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        //encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout



        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftFrontTarget;
        int     newRightFrontTarget;
        int     newLeftBackTarget;
        int     newRightBackTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;
        
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftFrontTarget = robot.motorleftfront.getCurrentPosition() + moveCounts;
            newRightFrontTarget = robot.motorrightfront.getCurrentPosition() + moveCounts;
            newLeftBackTarget = robot.motorleftback.getCurrentPosition() + moveCounts;
            newRightBackTarget = robot.motorrightback.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.motorleftfront.setTargetPosition(newLeftFrontTarget);
            robot.motorrightfront.setTargetPosition(newRightFrontTarget);
            robot.motorleftback.setTargetPosition(newLeftBackTarget);
            robot.motorrightback.setTargetPosition(newRightBackTarget);

            robot.motorleftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorrightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorleftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorrightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.motorleftfront.setPower(speed);
            robot.motorrightfront.setPower(speed);
            robot.motorleftback.setPower(speed);
            robot.motorrightback.setPower(speed);

            // keep looping while we are still active, and ALL motors are running.
            while (opModeIsActive() &&
                    (robot.motorleftfront.isBusy()
                            && robot.motorrightfront.isBusy()
                            && robot.motorleftback.isBusy()
                            && robot.motorrightback.isBusy()
                    )) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.motorleftfront.setPower(leftSpeed);
                robot.motorrightfront.setPower(rightSpeed);
                robot.motorleftback.setPower(leftSpeed);
                robot.motorrightback.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftFrontTarget,  newRightFrontTarget);
                telemetry.addData("Actual",  "%7d:%7d:7d:%7d", robot.motorleftfront.getCurrentPosition(),
                        robot.motorrightfront.getCurrentPosition(),
                        robot.motorleftback.getCurrentPosition(),
                        robot.motorrightback.getCurrentPosition()
                );
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.motorleftfront.setPower(0);
            robot.motorrightfront.setPower(0);
            robot.motorleftback.setPower(0);
            robot.motorrightback.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.motorleftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorrightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorleftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorrightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.motorleftfront.setPower(0);
        robot.motorrightfront.setPower(0);
        robot.motorleftback.setPower(0);
        robot.motorrightback.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.motorleftfront.setPower(leftSpeed);
        robot.motorrightfront.setPower(rightSpeed);
        robot.motorleftback.setPower(leftSpeed);
        robot.motorrightback.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }





    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.motorleftfront.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.motorrightfront.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.motorleftback.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightBackTarget = robot.motorrightback.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.motorleftfront.setTargetPosition(newLeftFrontTarget);
            robot.motorrightfront.setTargetPosition(newRightFrontTarget);
            robot.motorleftback.setTargetPosition(newLeftBackTarget);
            robot.motorrightback.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.motorleftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorrightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorleftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorrightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.motorleftfront.setPower(Math.abs(speed));
            robot.motorrightfront.setPower(Math.abs(speed));
            robot.motorleftback.setPower(Math.abs(speed));
            robot.motorrightback.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that all motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.motorleftfront.isBusy()
                           && robot.motorrightfront.isBusy()
                           && robot.motorleftback.isBusy()
                           && robot.motorrightback.isBusy()
                   )) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,
                                                                                newRightFrontTarget,
                                                                                newLeftBackTarget,
                                                                                newRightBackTarget
                                );
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            robot.motorleftfront.getCurrentPosition(),
                                            robot.motorrightfront.getCurrentPosition(),
                                            robot.motorleftback.getCurrentPosition(),
                                            robot.motorrightback.getCurrentPosition()
                );
                telemetry.update();
            }

            // Stop all motion;
            robot.motorleftfront.setPower(0);
            robot.motorrightfront.setPower(0);
            robot.motorleftback.setPower(0);
            robot.motorrightback.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.motorleftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorrightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorleftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorrightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
            //  sleep(250);   // optional pause after each move
        }
    }
}
