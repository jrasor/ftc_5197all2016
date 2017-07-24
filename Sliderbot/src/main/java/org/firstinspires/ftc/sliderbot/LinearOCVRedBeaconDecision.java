package org.firstinspires.ftc.sliderbot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Mat;
import org.opencv.core.Size;

/**
 Basic Vision application.
 *
 * This opmode looks at beacon, decides which button to push for the Red
 *   Alliance, and pushes it. Image processing by OpenCV.
 *
 * A LinearVisionOpMode allows using Vision Extensions, which do a lot of processing
 * for you. Just enable the extension and set its options to your preference!
 *
 *   This version can run without a robot. Just run it from the Driver
 *     station, point the Robot Controller phone at a beacon, and see what it
 *     says: "Hmm, looking", "Push left button", "Push right button".
 *   Randomize the beacon several times, and see how good the decisions are.
 *     With a captured beacon, the decision may be silly, but who cares? In
 *     Autonomous, we won't go after captured beacons. We will in TeleOp, but
 *     by then the Driver sees the beacons and doesn't need this to decide.
 *
 *     WORKS.
 *
 * Version history
 *   1/5/17 JMR version 0.1: TeleOp mode, announces decision but cannot act
 *     on it. Button push code is stubbed. Some later version will be Auto-
 *     nomous.
 *   1/19/17 JMR version 0.2: renamed, adapted for Sliderbot class.

 */

@TeleOp(name="Linear Red Beacon Decision", group="Sliderbot")
//@Disabled
public class LinearOCVRedBeaconDecision extends LinearVisionOpMode {

    //Frame counter
    int frameCount = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        //Wait for vision to initialize - this should be the first thing you do
        waitForVisionStart();

        /**
         * See Basic Vision Sample for tips on setting these options.
         **/

        this.setCamera(Cameras.PRIMARY);
        this.setFrameSize(new Size(900, 900));
        enableExtension(Extensions.BEACON);         //Beacon detection
        enableExtension(Extensions.ROTATION);       //Automatic screen rotation correction
        enableExtension(Extensions.CAMERA_CONTROL); //Manual camera control
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);

        // Color tolerances: 0 is default, -1 is minimum and 1 is maximum.
        beacon.setColorToleranceRed(0);
        beacon.setColorToleranceBlue(0);
        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.PORTRAIT);
        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();

        //Wait for the Driver to press Play
        waitForStart();

        //Main loop
        //Camera frames and OpenCV analysis will be delivered to this method as quickly as possible
        //This loop will exit once the opmode is closed
        while (opModeIsActive()) {
            //  Vuforia initialization and driving code can go here

            //You can access the most recent frame data and modify it here using getFrameRgba() or getFrameGray()
            //Vision will run asynchronously (parallel) to any user code so your programs won't hang
            //You can use hasNewFrame() to test whether vision processed a new frame
            //Once you copy the frame, discard it immediately with discardFrame()
            if (hasNewFrame()) {
                //Get the frame
                Mat rgba = getFrameRgba();
                String action = "No decision yet.";
                String leftColor = beacon.getAnalysis().getColorString();
                leftColor = leftColor.substring(0,3);

                if (leftColor.equals("red")) { action = "Push left button."; } // Red robot
                if (leftColor.equals("blu")) { action = "Push right button."; }
                if (leftColor.equals("???")) { action = "Hmm, looking..."; }

                telemetry.addData("Left", leftColor);
                telemetry.addData("Beacon Center", beacon.getAnalysis().getLocationString());
                telemetry.addData("Confidence", beacon.getAnalysis().getConfidenceString());
                telemetry.addData("Decision", action);
                telemetry.addData("Frame Counter", frameCount);
                telemetry.update();
                //Discard the current frame to allow for the next one to render
                discardFrame();

                //Do all of your custom frame processing here
                //For this opmode, let's just add to a frame counter
                frameCount++;
            }

            //Wait for a hardware cycle to allow other processes to run
            waitOneFullHardwareCycle();
        }
    }
}
