package org.firstinspires.ftc.gameday3;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.VisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;

/**
 * Basic Vision application.
 * <p/>
 * This opmode looks at beacon, decides which button to push for the Red
 *   Alliance, and pushes it. Image processing by OpenCV.
 *
 * Version history
 *   1/5/17 JMR version 0.1: TeleOp mode, announces decision but cannot act
 *     on it. Button push code is stubbed. Some later version will be Auto-
 *     nomous.
 *   This version can run without a robot. Just run it from the Driver
 *     station, point the Robot Controller phone at a beacon, and see what it
 *     says: "No decision", "pushing left button", "pushing right button".
 *   Randomize the beacon several times, and see how good the decisions are.
 *     With a captured beacon, the decision may be silly, but who cares? In
 *     Autonomous, we won't go after captured beacons. We will in TeleOp, but
 *     by then the Driver sees the beacons and doesn't need this to decide.
 *
 *     WORKS.
 *
 *   1/19/17 JMR version 0.2: renamed, makes decision for the Red Alliance.
 */

@TeleOp(name="OCV Red Beacon Decision", group="Meet 3")
//@Disabled
public class OCVRedBeaconDecision extends VisionOpMode {

    @Override
    public void init() {
        super.init();

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
    }

    @Override
    public void loop() {
        super.loop();
        String action = "No decision yet.";
        String leftColor = beacon.getAnalysis().getColorString();
        leftColor = leftColor.substring(0,3);

        if (leftColor.equals("red")) { action = "Push left button."; } // Red robot
        if (leftColor.equals("blu")) { action = "Push right button."; }
        if (leftColor.equals("???")) { action = "Hmm, looking..."; }

        telemetry.addData("Beacon Left Color", leftColor);
        telemetry.addData("Beacon Center", beacon.getAnalysis().getLocationString());
        telemetry.addData("Beacon Confidence", beacon.getAnalysis().getConfidenceString());
        telemetry.addData("Decision", action);
    }

    @Override
    public void stop() {
        super.stop();
    }
}
