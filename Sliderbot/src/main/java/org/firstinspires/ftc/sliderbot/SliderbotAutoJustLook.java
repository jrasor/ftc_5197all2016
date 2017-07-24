package org.firstinspires.ftc.sliderbot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 *  Tour all 4 beacons, identifying the Vuforia images below them.
 *
 *  JMR 2/10/17 v 0.1 3/4 WORKS. ** 600ms is not enough wait time, usually misses a report.
 */

@Autonomous(name="Sliderbot Tour 4 Beacons", group="Testing")
//@Disabled
public class SliderbotAutoJustLook extends LinearOpMode {

    /* Declare OpMode members. */
    //public VVField field = new VVField(this);
    Sliderbot robot = new Sliderbot(this);
    private final double STRAIGHT_SPEED = 0.70;
    private final double TURN_SPEED = 0.70;
    private final double TURN_RATIO = 0.40;
    private final double SEARCH_SPEED = 0.30;
    private final int WAIT_TIME = 600;
    String report = "I see ";
    String imageName = "nothing";

    @Override
    public void runOpMode() {
        robot.initEncodedDrive(hardwareMap);
        //  Get an initialized VuforiaLocalizer.
        VuforiaLocalizer vuforia = robot.initVuforia();
        //  Get the beacon images trackable and located.
        robot.allTrackables = robot.initBeaconImages(vuforia);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status: ", "Ready!");
        telemetry.update();
        waitForStart();

        jobRunScript();
    }

    // Step through each leg of the path.
    // Note: Reverse movement is obtained by setting a negative distance (not speed)
    public void jobRunScript(){
        // Place robot's left edge 2' from left corner, 2' from wall, or at Field corner
        //   of the Red Corner Vortex tile. This is one tile diagonally different from
        //   placement as used for Autonomous beacon capture for Red Alliance.

        // Run straight forward to put first beacon on port beam...
        robot.driveStraight(STRAIGHT_SPEED, 26.0);
        // ... Turn port to approximately face first beacon image...
        robot.turnAngle (TURN_SPEED, Math.PI/2);
        sleep(WAIT_TIME);
        imageName = robot.findBeaconImage(robot.allTrackables);
        report += imageName;
        telemetry.addData("", report);
        telemetry.update();
        report = " ";
        // ... turn starboard, ready to move to next observation point.
        robot.turnAngle(TURN_SPEED, -Math.PI/2);

        // Move to next observation point, look, and report.
        robot.driveStraight(STRAIGHT_SPEED, 48.0);
        robot.turnAngle (TURN_SPEED, Math.PI/2);
        sleep(WAIT_TIME);
        imageName = robot.findBeaconImage(robot.allTrackables);
        report += imageName;
        telemetry.addData("", report);
        telemetry.update();
        report = " ";

        // Move to third observation point, look, and report.
        robot.turnAngle (TURN_SPEED, -Math.PI/2);
        sleep(WAIT_TIME);
        imageName = robot.findBeaconImage(robot.allTrackables);
        report += imageName;
        telemetry.addData("", report);
        telemetry.update();
        report = " ";

        // Move to fourth observation point, look, and report.
        robot.turnAngle (TURN_SPEED, -Math.PI/2);
        robot.driveStraight(STRAIGHT_SPEED, 48.0);
        robot.turnAngle(TURN_SPEED, Math.PI/2);
        sleep(WAIT_TIME);
        imageName = robot.findBeaconImage(robot.allTrackables);
        report += imageName;
        telemetry.addData("", report);
        telemetry.update();
        report = " ";
        sleep (5000);
    }
}
