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
package org.firstinspires.ftc.vision;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
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
 */

@TeleOp(name="Look For Beacon", group ="Vuforia")
//@Disabled
public class VuforiaLookForBeacon extends LinearOpMode {
    // Op mode properties
    public VVField field = new VVField(this);
    public Runnerbot robot = new Runnerbot(this);
    OpenGLMatrix lastLocation = null;

    @Override public void runOpMode() {

        // Logging
        final String TAG = "Go To Beacon";
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //  Get an initialized VuforiaLocalizer.
        VuforiaLocalizer vuforia = robot.initVuforia();

        //  Get the beacon images trackable and located.
        List<VuforiaTrackable> allTrackables = robot.initBeaconImages(vuforia);

        /** Wait for the driver to press the Play button on gamepad. */
        waitForStart();

        while ( opModeIsActive() ) {
            String field = "I see";
            String value = "";
            boolean seeSomething = false;

            //  Find an image, and get robot's location relative to it.
            for (VuforiaTrackable trackable : allTrackables) {
                /**
                 * getUpdatedRobotLocation() will return null if no new information is available since
                 * the last time that call was made, or if the trackable is not currently visible.
                 * getRobotLocation() will return null if the trackable is not currently visible.
                 */
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()){
                    seeSomething = true;
                    value = trackable.getName();
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                        break; // Never interested in more than one image.
                    }
                }
            }
            /**
             * Report where the robot was last located (if we know).
             */
            if (seeSomething) {
                //  Line up on it, and go towards it
                VectorF translation = lastLocation.getTranslation();
                double x = -translation.get(0);
                double y = -translation.get(1);
                double targetY = robot.CAMERA_FROM_FRONT; // Millimeters. Stop when camera is 12" from image.
                double remainingY = y - targetY; // Millimeters away from stopping.
                // Guidance tuning constants
                final double HEADING_CORRECTOR = 3.0; // Controls strength of X correction
                /* Angles in radians */
                // Zero if camera directly facing plane of the image, independent of camera position.
                // Positive if pointing to left of normal to that plane through the camera.
                double robotHeading;
                // Zero if camera anywhere on normal to image, independent of camera orientation.
                // Positive if image is left of normal to plane of image, passing through the camera.
                double imageBearing;
                // approximation: atan2(y, x) is close to x/y for abs(x) << abs(y).
                imageBearing = -x/y;
                imageBearing *= 180/Math.PI;
                double targetHeading = imageBearing * HEADING_CORRECTOR;
                value += String.format(" at X, Y: %6.0fmm",x) + String.format(", %6.0fmm", y);
                value += String.format("\nImage bearing %6.0f degrees", imageBearing);
                Orientation orientation = Orientation.getOrientation(lastLocation,
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                robotHeading = orientation.thirdAngle;
                value += String.format("\nRobot heading: %6.0f degrees ", robotHeading);

                // Pretend to drive robot like an airplane landing on a runway. We try to
                //   be on its long axis, and pointed along that axis.
                // Suggest a command to steer toward this heading, attempting to line up
                if (robotHeading < targetHeading) {
                    value += "Turning left to";
                }
                if (robotHeading > targetHeading) {
                    value += "Turning right to";
                }
                if (robotHeading == targetHeading) {
                    value += "Keeping straight on";
                }
                value += String.format(" heading %6.0f.", targetHeading);
                value += String.format(" %6.0fmm to go.", remainingY);
                if (remainingY < 0) {
                    // Just stop, and let driver contemplate the situation.
                    value += " I'm too close. Stopping.";
                    //sleep (5000);
                }
            } else {
                value = "nothing. Stopping.";
                //seeSomething = false;
            }

            telemetry.addData(field, value);
            telemetry.update();
        }
    }
}