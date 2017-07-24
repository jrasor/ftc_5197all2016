/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
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

Copyright (c) 2016, Don Bosco Technical Institute FTC Robotics Team 5197

 This code is based on the work of Robert Atkinson, and is distributed under the
 same conditions as his original copyright notice.

*/
package org.firstinspires.ftc.sliderbot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;

/**
 * This OpMode illustrates the basics of using the Vuforia localizer to help a robot
 *   autonomously navigate close to and directly facing a Velocity Vortex beacon image.
 *   Even though this is a TeleOp mode, it can do the autonomous navigation on the
 *   driver's command.
 * It is derived from opmode just Drive, but adds support for Vuforia robot vision, a
 *   Particle sweeper/shooter, a beacon button pusher, and uses gamepad 1's A
 *   button to invoke a script of movements and actuator commands. Such a script
 *   can be thought of as a robot macro.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * IMPORTANT: In order to use this OpMode, you need to get your own Vuforia license key.
 *
 * Version history
 *   0.1 - Shane's adaptation of "SlideSweepPushMacro" (v 0.7) made to be more basic for demonstration
 */

@TeleOp(name="Sliderbot Basic Teleop", group ="Sliderbot")
//@Disabled
public class SliderbotBasicTeleop extends LinearOpMode {
    // Op mode properties
    //public VVField field = new VVField(this);
    public Sliderbot robot = new Sliderbot(this);
    public static final boolean ENCODED = true;
    public static final boolean UNENCODED = true;


    @Override
    public void runOpMode() {

        /***              Start up the robot.             ***/
        robot.initUnencodedDrive(hardwareMap);
        robot.initBeaconPusher();
        robot.initSweeper();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        /** Wait for the driver to press the Play button on gamepad. */
        waitForStart();
        // ** Could Vuforia initialization go here? Team will drive the robot
        //    around some before invoking the approach macro.
        while (opModeIsActive()) {
            telemetry.addData ("", robot.justDrive());
            telemetry.update();
            // Operate the sweeper
            if(gamepad1.right_bumper)
            {
                robot.sweepToggleState = 1;
                robot.sweepToggle = true;
            }
            else if(gamepad1.left_bumper)
            {
                robot.sweepToggleState = 2;
                robot.sweepToggle = true;
            }

            if(gamepad1.b)
            {
                robot.sweepToggle = false;
            }

            if(robot.sweepToggle)
            {
                switch (robot.sweepToggleState) {
                    case 0: // off
                        robot.sweepMotor.setPower(0);
                        break;
                    case 1: // shoot
                        robot.sweepMotor.setPower(robot.sweepSpeed);
                        break;
                    case 2: // suck
                        robot.sweepMotor.setPower(-robot.sweepSpeed / 2);
                        break;
                }
            }
            else
            {
                robot.sweepMotor.setPower(0);
            }

            // Operate the beacon button pusher
            if(gamepad1.dpad_left)
            {
                robot.buttonPush.setPosition(1);
            }
            if(gamepad1.dpad_right)
            {
                robot.buttonPush.setPosition(0);
            }
        }
    }
}