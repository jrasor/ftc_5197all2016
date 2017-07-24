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
package org.firstinspires.ftc.gameday3;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

/**
 * This OpMode illustrates the basics of using the Vuforia localizer to help a robot
 *   autonomously navigate close to and directly facing a Velocity Vortex beacon image.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * IMPORTANT: In order to use this OpMode, you need to get your own Vuforia license key.
 *
 * Hardware needed: a robot with left and right drive. No servos or sensors, other then
 *   the Robot Controller phone, are necessary.
 *
 * Version history
 *   version 0.1  JMR 12/28/16: Adapted from VuforiaNavigation06, but puts
 *     all the images at the origin and facing the Red Alliance Wall.
 *   version 0.2 JMR 1/4/17 Coded as a TeleOp. Press A button on gamepad 1. The robot
 *     goes into the image, directly facing it, and 12" away from the camera. Then it
 *     turns 90 degrees to present its portside sensors and actuators to the beacon.
 *   version 0.3 JMR 1/9/17. Driver can drive around, then press A on gamepad1 and
 *     go to nearby beacon, and stop 8" from it. No turn after that. No sweeper, no
 *     beacon pusher. WORKS.
 *   version 0.4 JMR 1/13/17. Sweeper, beacon pusher added. WORKS.
 *   version 0.5 JMR 1/14/17. A button macro: approach beacon, turn pusher toward it
 *     added. WORKS.
 */

@TeleOp(name="Slide Sweep Push Macro", group ="Meet 3")
//@Disabled
public class SlideSweepPushMacro extends LinearOpMode {
    // Op mode properties
    //public VVField field = new VVField(this);
    public Sliderbot robot = new Sliderbot(this);
    public static final String TAG = "Go to beacon";

    @Override
    public void runOpMode() {

        // Logging
        final String TAG = "Go To Beacon";

        // Start up the robot.
        robot.initDrive(hardwareMap);
        robot.initEncodedDrive();
        robot.initBeaconPusher();
        robot.initSweeper();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //  Get an initialized VuforiaLocalizer.
        VuforiaLocalizer vuforia = robot.initVuforia();

        //  Get the beacon images trackable and located.
        robot.allTrackables = robot.initBeaconImages(vuforia);

        /** Wait for the driver to press the Play button on gamepad. */
        waitForStart();

        while (opModeIsActive()) {
            robot.justDrive();

            //   Invoke beacon approach macro.
            if (gamepad1.a) {
                robot.goToBeacon();
                robot.turnOnSelf(0.15, Math.PI/4);
                robot.initDrive(hardwareMap);
                robot.initEncodedDrive();
                //  A 90 degree starboard (CW) turn here will present the port side
                //    to the beacon.
                //  The gamepad1.a event handler could invoke a whole script, like
                //    goToBeacon
                //    turn 90 degrees
                //    punch any button. In Teleop, both are typically the same color,
                //    opposite our Alliance. Otherwise, we wouldn't be driving up
                //    to this beacon.
                //  Macros or scripts could be assigned to other gamepad controls, like
                //  if (gamepad1.b)
                //    turn around
                //    drive away from beacon
            }

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
                        robot.sweepMotor.setPower(-robot.sweepSpeed / 4);
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

    }//

}//