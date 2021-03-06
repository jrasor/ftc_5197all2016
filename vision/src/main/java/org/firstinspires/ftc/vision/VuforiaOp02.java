package org.firstinspires.ftc.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.vision.R;
import org.firstinspires.ftc.Sliderbot;

/**
 * Adapted from top comment and reply of https://www.youtube.com/watch?v=2z-o9Ts8XoE.
 * Thanks to Team 3491, "Fixit".
 *

 Copyright (c) 2016 Don Bosco Technical Institute Robotics, FTC Team 5197

 All rights reserved.

 Redistribution and use in source and binary forms, with or without modification,
 are permitted (subject to the limitations in the disclaimer below) provided that
 the following conditions are met:

 Redistributions of source code must retain the above copyright notice, this list
 of conditions and the following disclaimer.

 Redistributions in binary form must reproduce the above copyright notice, this
 list of conditions and the following disclaimer in the documentation and/or
 other materials provided with the distribution.

 Neither the name of Don Bosco Technical Institute nor the names of his contributors
 may be used to endorse or promote products derived from this software without specific
 prior written permission.

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

 * Revision history
 *
 * v 0.1    11/21/16 JMR only changed comments like this one, and the two in the
 *   for(VuforiaTrackable) loop. This version does not need a robot to run, just a
 *   Robot Controller phone and a Driver phone.
 * v 0.2    11/22/16 JMR reformatted position and heading report. Set name not "Lego" but
 * "Legos".
 * v 0.3    12/27/16 JMR changed it to a TeleOp mode.
 */

@TeleOp(name="Vuforia Navigation 3491-02", group ="Vuforia")
//@Disabled
public class VuforiaOp02 extends LinearOpMode {
    public VVField field = new VVField(this);
    public Runnerbot robot = new Runnerbot(this);
    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "ASkv3nr/////AAAAGYZ9CexhH0K0lDbV090F719DkwXCIXEUmExgnQNDFGjrDrkVJnU7xNhuKHLsC32Pb1jmr+6vp6JtpVKvNmTf28ZYkUphDeajNPCLgGVxLjD6xsfgBayqSO9bfQFeGkrdEgXlP+2oaz234afhWti9Jn8k71mzbQ4W2koX9yBMWz0YLzUWClcasxi6Nty7SUvV+gaq3CzpKVtjKk+2EwV6ibIc0V47LAeB0lDGsGkSzuJ+93/Ulpoj+Lwr/jbI2mu/Bs2W7U9mw73CMxvDix9o1FxyPNablla4W5C5lUDm0j2lW5gsUNOhgvlWKQ+eCu9IBp53WbW5nfNzhXPaDDh/IlBbZuAMIJuMDEHI5PVLKT9L";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Legos"); // Was "Lego" in v. 0.1
        beacons.get(3).setName("Gears");

        waitForStart();

        beacons.activate();

        while (opModeIsActive()){
            for(VuforiaTrackable beac : beacons){
                // 4 x 4 transformation matrix
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();

                if(pose != null){
                    VectorF translation = pose.getTranslation();
                    // Report x, y, and z of image relative to phone.
                    telemetry.addData(beac.getName() + " image is ", "%4.0fmm in front of me, and\n%4.0fmm to the side.",
                            -translation.get(2), translation.get(0));
                    // Degrees to turn robot in order to face directly toward the recognized image
                    // below the beacon.
                    double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(0), -translation.get(2)));
                       //     Math.atan2(translation.get(1), translation.get(2));
                    telemetry.addData("Relative bearing ", "%4.1f degrees.", degreesToTurn);
                }
            }
            telemetry.update();
        }
    }
}
