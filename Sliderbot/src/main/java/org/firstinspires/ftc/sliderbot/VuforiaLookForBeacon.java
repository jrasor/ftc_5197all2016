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

 * Revision history
 *   version 0.1  JMR 12/28/16: Adapted from VuforiaNavigation06, but puts
 *     all the images at the origin and facing the Red Alliance Wall.
 *   version 0.2  JMR 12/30/16: robot uses its position relative to image to
 *     go to it and face it.
*/
package org.firstinspires.ftc.sliderbot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import java.util.List;

/**
 * This OpMode illustrates the basics of using the Vuforia localizer determine a
 *   robot's location with respect to a Velocity Vortex Beacon image. The "robot",
 *   in this OpMode, is simply the Robot Controller Phone. Print one of the images
 *   and stick it up on a wall, run this Opmode, and hold the Robot Controller
 *   phone up to look at it. The Driver Station phone will report the Controller
 *   phone's location relative to the image. It will also report what it would have
 *   told a real robot to do in response to that location.
 * No commands are issued to the robot drive train.
 *
 * IMPORTANT: use a robot configuration with nothing at all attached.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * IMPORTANT: In order to use this OpMode, you need to get your own Vuforia license key as
 * explained below.
 *
 * Version history
 *   v 0.1 JMR 2/4/17 WORKS.
 */

@TeleOp(name="Look For Beacon", group ="Sliderbot")
//@Disabled
// ** Fails to initialize camera if this extends LinearVisionOpmode
public class VuforiaLookForBeacon extends LinearOpMode {
    // Op mode properties
    public Sliderbot robot = new Sliderbot(this);
    OpenGLMatrix lastLocation = null;

    @Override public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //  Get an initialized VuforiaLocalizer.
        VuforiaLocalizer vuforia = robot.initVuforia();
        //  Get the beacon images trackable and located.
        telemetry.addData("Status", "Vuforia ready.");
        telemetry.update();
        List<VuforiaTrackable> allTrackables = robot.initBeaconImages(vuforia);
        telemetry.addData("Status", "Images trackable.");
        telemetry.update();
        /** Wait for the driver to press the Play button on gamepad. */
        waitForStart();

        while ( opModeIsActive() ) {
            String field = "I see";
            String imageName = "";

            //  Find an image, and get robot's location relative to it.
            imageName = robot.findBeaconImage(allTrackables);

            telemetry.addData(field, imageName);
            telemetry.update();
        }
    }
}