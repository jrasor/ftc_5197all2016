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
*/
package org.firstinspires.ftc.vision;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
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
 * This OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the FTC field.
 * The code is structured as a LinearOpMode
 *
 * Vuforia uses the phone's camera to inspect its surroundings, and attempt to locate target images.
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code than combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * This example assumes a "diamond" field configuration where the red and blue alliance stations
 * are adjacent on the corner of the field furthest from the audience.
 * From the Audience perspective, the Red driver station is on the right.
 * The two vision targets are located on the two walls closest to the audience, facing in.
 * Gears and Tools are on the RED side of the field, while Wheels and Legos are on the Blue side.
 *
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 *
 * Revision history
 *   version 0.1  JMR 11/21/16, unmodified from FTC SDK external sample ConceptVuforiaNavigation
 *   version 0.2  JMR 11/26/16, modified for 2016-2017 game objects.
 *   version 0.3  JMR 12/9/16, modified to use phone in portrait mode, middle of robot,
 *     back lens about 6" off the floor. It's now a TeleOp, so it won't time out during testing.
 *     Floats, no doubles.
 *   version 0.4  JMR 12/9/16, transformation formatted for better readability.
 *   version 0.5  JMR 12/10/16, reports which image found, robot XYZ position, and robot
 *     heading relative to +Y axis.
 *   version 0.6  JMR 12/10/16, used ordinary axes order XYZ.
 *   version 0.7  JMR 1/1/17, uses VVField and Runnerbot classes
 */

@TeleOp(name="Concept: Vuforia Navigation07", group ="Vuforia")
//@Disabled
public class VuforiaNavigation07 extends LinearOpMode {
    public VVField field = new VVField(this);
    public Runnerbot robot = new Runnerbot(this);
    public static final String TAG = "Vuforia Sample";

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override public void runOpMode() {
        /**
         * Start up Vuforia, telling it the id of the view that we wish to use as the parent for
         * the camera monitor feedback; if no camera monitor feedback is desired, use the
         * parameterless constructor instead. We also indicate which camera on the Robot Controller
         * that we wish to use: the back one.
         *
         * Note that in addition to indicating which camera is in use, we also need to tell the
         * system the location of the phone on the robot; see phoneLocationOnRobot below.
         *
         * IMPORTANT: You need to obtain your own license key to use Vuforia. Get a free one at
         * https://developer.vuforia.com/license-manager.
         *
         * Once you've obtained a license key, copy the string form of the key from the Vuforia
         * web site and paste it in to your code as the value of the 'vuforiaLicenseKey' field
         * of the {@link Parameters} instance with which you initialize Vuforia.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "ASkv3nr/////AAAAGYZ9CexhH0K0lDbV090F719DkwXCIXEUmExgnQNDFGjrDrkVJnU7xNhuKHLsC32Pb1jmr+6vp6JtpVKvNmTf28ZYkUphDeajNPCLgGVxLjD6xsfgBayqSO9bfQFeGkrdEgXlP+2oaz234afhWti9Jn8k71mzbQ4W2koX9yBMWz0YLzUWClcasxi6Nty7SUvV+gaq3CzpKVtjKk+2EwV6ibIc0V47LAeB0lDGsGkSzuJ+93/Ulpoj+Lwr/jbI2mu/Bs2W7U9mw73CMxvDix9o1FxyPNablla4W5C5lUDm0j2lW5gsUNOhgvlWKQ+eCu9IBp53WbW5nfNzhXPaDDh/IlBbZuAMIJuMDEHI5PVLKT9L";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data sets for the trackable objects we wish to track. These particular data
         * sets are stored in the 'assets' part of our application (you'll see them in the Android
         * Studio 'Project' view over there on the left of the screen). You can make your own
         * datasets with the Vuforia Target Manager: https://developer.vuforia.com/target-manager.
         * PDFs for the example "FTC_2016-17", datasets can be found in in this project at
         * http://www.firstinspires.org/resource-library/ftc/game-and-season-info.
         */
        VuforiaTrackables beacons = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Legos");
        beacons.get(3).setName("Gears");

        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(beacons);

        /**
         * We use units of mm here because that's the recommended units of measurement for the
         * size values specified in the XML for the ImageTarget trackables in data sets. E.g.:
         *      <ImageTarget name="Wheels" size="254.000000 184.154922" />
         * You don't *have to* use mm here, but the units here and the units used in the XML
         * target configuration files *must* correspond for the math to work out correctly.
         */
        float mmPerInch         = 25.4f;
        float mmPerFoot         = 12 * mmPerInch;
        float mmBotWidth        = 17.8f * mmPerInch;          // Big Tile Runner
        float mmBotLength       = 18.0f * mmPerInch;
        float mmFTCFieldWidth   = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10"
        // center-to-center of the glass panels, which are an inch inside the 12'x12' arena.
        float mmImageHeight     = 146; // 1.5 + 8.5/2 inches off floor

        /**
         * In order for localization to work, we need to tell the system where each target we
         * wish to use for navigation resides on the field, and we need to specify where the phone
         * sits on the robot. These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * For the most part, you don't need to understand the details of the math of how transformation
         * matrices work inside (as fascinating as that is, truly). Just remember these key points:
         * <ol>
         *
         *     <li>You can put two transformations together to produce a third that combines the effect of
         *     both of them. If, for example, you have a rotation transform R and a translation transform T,
         *     then the combined transformation matrix RT which does the rotation first and then the translation
         *     is given by {@code RT = T.multiplied(R)}. That is, the transforms are multiplied in the
         *     <em>reverse</em> of the chronological order in which they applied.</li>
         *
         *     <li>A common way to create useful transforms is to use methods in the {@link OpenGLMatrix}
         *     class and the Orientation class. See, for example, {@link OpenGLMatrix#translation(float,
         *     float, float)}, {@link OpenGLMatrix#rotation(AngleUnit, float, float, float, float)}, and
         *     {@link Orientation#getRotationMatrix(AxesReference, AxesOrder, AngleUnit, float, float, float)}.
         *     Related methods in {@link OpenGLMatrix}, such as {@link OpenGLMatrix#rotated(AngleUnit,
         *     float, float, float, float)}, are syntactic shorthands for creating a new transform and
         *     then immediately multiplying the receiver by it, which can be convenient at times.</li>
         *
         *     <li>If you want to break open the black box of a transformation matrix to understand
         *     what it's doing inside, use {@link MatrixF#getTranslation()} to fetch how much the
         *     transform will move you in x, y, and z, and use {@link Orientation#getOrientation(MatrixF,
         *     AxesReference, AxesOrder, AngleUnit)} to determine the rotational motion that the transform
         *     will impart. See {@link #format(OpenGLMatrix)} below for an example.</li>
         *
         * </ol>
         *
         * This example places the four target images below the Velocity Vortex beacons.
         * See the doc folder of this project for a description of the field Axis conventions.
         *
         * Initially the target is assumed lying at the origin of the field's coordinate system
         * (the center of the field), facing up. In this configuration, the target's coordinate
         * system aligns with that of the field.
         *
         * To place the Tools Target on the Red Audience wall:
         *   First we rotate it 90 around the field's X axis to flip it upright.
         *   Then we rotate it  90 around the field's Z axis to face away from the audience.
         *   Finally, we translate it 6 feet along the X axis towards the Red Audience wall, and
         *   3 feet along the +Y axis toward the Audience corner, and 160 mm up off the floor.
         * The code applies these transformations in reverse order: translation, rotation.
         */
        OpenGLMatrix toolsTargetLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth/2, 3*mmPerFoot, mmImageHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, 90));
        beacons.get(1).setLocation(toolsTargetLocationOnField);
        RobotLog.ii(TAG, "Tools=%s", format(toolsTargetLocationOnField));

        /* To place the Gears Target on the Red Audience wall:
        *    First we rotate it 90 around the field's X axis to flip it upright.
        *    Then we rotate it  90 around the field's Z axis to face away from the audience.
        *    Finally, we translate it 6 feet along the X axis towards the Red Audience wall, and
        *    1 foot along the -Y axis away from the Audience corner, and 160 mm up off the floor.
        *  The code applies these transformations in reverse order: translation, rotation.
        */
        OpenGLMatrix gearsTargetLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth/2, -mmPerFoot, mmImageHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, 90));
        beacons.get(3).setLocation(gearsTargetLocationOnField);
        RobotLog.ii(TAG, "Gears=%s", format(gearsTargetLocationOnField));

        /* To place the Legos Target on the Blue Audience wall:
        *    First we rotate it 90 around the field's X axis to flip it upright.
        *    We need no other rotation, since it is already facing directly away from the Blue
        *    Audience wall.
        *    Finally, we translate it 3' along -X axis, then 6' (half arena width) along +Y
        *    axis, and 160mm up off the floor.
        *  The code applies these transformations in reverse order: translation, rotation.
        */
        OpenGLMatrix legosTargetLocationOnField = OpenGLMatrix
                .translation(-3*mmPerFoot, mmFTCFieldWidth/2, mmImageHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, 0));
        beacons.get(2).setLocation(legosTargetLocationOnField);
        RobotLog.ii(TAG, "Legos Target=%s", format(legosTargetLocationOnField));

        /* To place the Wheels Target on the Blue Audience wall:
        *    First we rotate it 90 around the field's X axis to flip it upright.
        *    We need no other rotation, since it is already facing directly away from the Blue
        *    Audience wall.
        *    Finally, we translate it 1 foot along -X axis, then 6 feet (half arena width) along +Y
        *    axis, and 160mm up off the floor.
        *  The code applies these transformations in reverse order: translation, rotation.
        */
        OpenGLMatrix wheelsTargetLocationOnField = OpenGLMatrix
                .translation(mmPerFoot, mmFTCFieldWidth/2, mmImageHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, 0));
        beacons.get(0).setLocation(wheelsTargetLocationOnField);
        RobotLog.ii(TAG, "Wheels Target=%s", format(legosTargetLocationOnField));
        /**
         * Create a transformation matrix describing where the phone is on the robot. Here, we
         * put the phone on the front middle of the robot with the screen facing in (see our
         * choice of BACK camera above) and in portrait mode. This requires no rotation of axes.
         *
         * When determining whether a rotation is positive or negative, consider yourself as looking
         * down the (positive) axis of rotation from the positive towards the origin. Positive rotations
         * are then CCW, and negative rotations CW. An example: consider looking down the positive Z
         * axis towards the origin. A positive rotation about Z (ie: a rotation parallel to the the X-Y
         * plane) is then CCW, as one would normally expect from the usual classic 2D geometry.
         */
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(0.0f, 0.0f, 0.0f); // Just locate the phone for now
        RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));

        /**
         * Let the trackable listeners we care about know where the phone is. We know that each
         * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
         * we have not ourselves installed a listener of a different type.
         */
        ((VuforiaTrackableDefaultListener)beacons.get(0).getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)beacons.get(1).getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)beacons.get(2).getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)beacons.get(3).getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

        /**
         * A brief tutorial: here's how all the math is going to work:
         *
         * C = phoneLocationOnRobot  maps   phone coords -> robot coords
         * P = tracker.getPose()     maps   image target coords -> phone coords
         * L = wheelsTargetLocationOnField maps   image target coords -> field coords
         *
         * So
         *
         * C.inverted()              maps   robot coords -> phone coords
         * P.inverted()              maps   phone coords -> imageTarget coords
         *
         * Putting that all together,
         *
         * L x P.inverted() x C.inverted() maps robot coords to field coords.
         *
         * @see VuforiaTrackableDefaultListener#getRobotLocation()
         */

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to initialize");
        telemetry.update();
        waitForStart();

        /** Start tracking the data sets we care about. */
        beacons.activate();

        while (opModeIsActive()) {
            String field = "Tracking";
            String value = " ";
            for (VuforiaTrackable trackable : allTrackables) {
                /**
                 * getUpdatedRobotLocation() will return null if no new information is available since
                 * the last time that call was made, or if the trackable is not currently visible.
                 * getRobotLocation() will return null if the trackable is not currently visible.
                 */
                value += ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ?
                        trackable.getName() : "";
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
            }
            /**
             * Provide feedback as to where the robot was last located (if we know).
             */
            if (lastLocation != null) {
                //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
                value += formatVuforiaTransform(lastLocation);
            } else {
                value += "Unknown";
            }
            telemetry.addData(field, value);
            telemetry.update();
        }
    }

    public String formatVuforiaTransform(OpenGLMatrix someTransformation,
                                         AxesReference axesReference, AxesOrder axesOrder, AngleUnit unit)
    {
        String result = "\n   Position:   ";
        VectorF translation = someTransformation.getTranslation();
        result=result.concat(String.format("  X: %6.0f", translation.get(0)));
        result=result.concat(String.format("  Y: %6.0f", translation.get(1)));
        result=result.concat(String.format("  Z: %6.0f mm", translation.get(2)));

        /**
         * An easy way to understand what a transform does is to look at the location
         * to which it transforms the origin of the coordinate system. Calling getTranslation()
         * carries out an equivalent computation as it extracts the translational aspect.
         */

        /**
         * Figure out in which direction we'd be looking after the transformation. Note that
         * the decomposition of a transformation into orientation angles can be subtle. See
         * {@link Orientation} for a full discussion.
         */

        result=result.concat ("\n   Heading: ");
        Orientation orientation = Orientation.getOrientation(someTransformation,
                axesReference, axesOrder, unit);
        if (orientation.angleUnit == AngleUnit.DEGREES)
            result=result.concat(String.format("%6.0f degrees", orientation.thirdAngle));
        else
            result=result.concat(String.format("%6.3f radians", orientation.thirdAngle));

        return result;
    }

    String formatVuforiaTransform(OpenGLMatrix someTransformation) {
        return formatVuforiaTransform(someTransformation,
                AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
    }
    /**
     * A simple utility that extracts positioning information from a transformation matrix
     * and formats it in a form palatable to a human being.
     */
    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }
}
