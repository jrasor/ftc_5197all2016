package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 *  Version history
 *
 *  version 0.1 JMR 11/27/16. A dead reckoning competition opmode. It does not use
 *    any sensor input, but relies on the motor encoder and calibration done using
 *    opmode "RangerbotEncodedMove10".
 *
 *  This file implements an Autonomous opmode for the 2016-2017 game, "Velocity
 *  Vortex". It aims to score by pushing the Cap Ball off the Center Vortex base,
 *  and by parking on that base. It does not attempt to score by shooting particles
 *  or claiming beacons.
 *
 *  No gamepad input is necessary, since this is an Autonomous opmode. Just put the robot
 *  with drive wheels front, to the right of the Red Alliance ally robot, about 42 inches
 *  from the corner. Then run the opmode, sit back and watch the show.
 *
 Copyright (c) 2016 Don Bosco Tech Robotics, FTC Team 5197

 All rights reserved.

 Redistribution and use in source and binary forms, with or without modification,
 are permitted (subject to the limitations in the disclaimer below) provided that
 the following conditions are met:

 Redistributions of source code must retain the above copyright notice, this list
 of conditions and the following disclaimer.

 Redistributions in binary form must reproduce the above copyright notice, this
 list of conditions and the following disclaimer in the documentation and/or
 other materials provided with the distribution.

 Neither the name of Don Bosco Tech nor the names of his contributors may be used to
 endorse or promote products derived from this software without specific prior
 written permission.

 NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */


@Autonomous(name="Push Cap Ball and Park", group="Rangerbot")
//@Disabled
public class PushCapBallPark extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareRangerbot robot = new HardwareRangerbot();   // Use a Rangerbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double LONG_TIME = 1.0e10;             // avoid timeout on encoderDrive
    static final double COUNTS_PER_MOTOR_REV = 1120.0;  // use 1440 for TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // 2.0 fr Pushbot, < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 2.9000; // determined by distance calibration.
    static final double RANGERBOT_DRIVEWHEEL_SEPARATION = 13.4115; // inches, by angle calibration.
    static final double ANGLE_SLIP_COMPENSATION = 0.20; // radians
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here.
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        idle();

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftMotor.getCurrentPosition(), // should be zero
                robot.rightMotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        jobRunScript();
    }

    //    Run some inches, and measure physical distance traveled.
    public void jobDistanceCalibrate (double inches)
    {
        straightDrive(DRIVE_SPEED, inches);
    }

    //    Turn through some angle, and measure physical change in orientation.
    public void jobAngleCalibrate (double angle)
    {
        turnDrive(DRIVE_SPEED, angle);
    }

    // Step through each leg of the path,
    // Note: Reverse movement is obtained by setting a negative distance (not speed)
    public void jobRunScript(){
        straightDrive(DRIVE_SPEED, 1.5);    // Adjust these ...
        turnDrive(TURN_SPEED, 0.4);         // ... to push cap ball off ...
        straightDrive(DRIVE_SPEED, 30.0);   // ... and park on Center Vortex base.
    }

    /*
     *  Method to perform an absolute move, based on encoder counts.
     *  Encoders are reset at the beginning of the move, ignoring the current position.
     *  Move will stop if any of three conditions occur:
     *  1) One motor gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            newLeftTarget = (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = (int) (rightInches * COUNTS_PER_INCH);
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));

            // While we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Goal", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Now at", "%7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    //    Wrapper for encoderDrive
    public void straightDrive(double speed, double inches) {
        encoderDrive(speed, inches, inches, LONG_TIME);
    }

    //    Wrapper for encoderDrive. Turns about axis halfway between drive wheels.
    public void turnDrive(double speed, double radians) {
        int turnTarget;

        turnTarget = (int)(radians * RANGERBOT_DRIVEWHEEL_SEPARATION / 2);
        encoderDrive(speed, -turnTarget, turnTarget, LONG_TIME);
    }
}
