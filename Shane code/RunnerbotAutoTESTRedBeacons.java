package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
/**
 *
 *  JMR 1/4/17 v. 0.1 go to Gears beacon, then turn right 90 degrees to present
 *    sensors and pusher to beacon buttons.
 */

@Autonomous(name="Sliderbot Red Auto Beacons", group="Runnerbot")
//@Disabled
public class RunnerbotAutoTESTRedBeacons extends LinearOpMode {

    /* Declare OpMode members. */
    //public VVField field = new VVField(this);
    public Sliderbot robot = new Sliderbot(this);

    @Override
    public void runOpMode() {
        robot.initEncodedDrive(hardwareMap);

        //  Get an initialized VuforiaLocalizer.
        VuforiaLocalizer vuforia = robot.initVuforia();

        //  Get the beacon images trackable and located.
        robot.allTrackables = robot.initBeaconImages(vuforia);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        jobRunScript();
    }

    // Step through each leg of the path,
    // Note: Reverse movement is obtained by setting a negative distance (not speed)
    public void jobRunScript(){
        // Place robot's left edge 4' from left corner, or exactly two tiles.
        // Run straight forward...
        robot.driveStraight(robot.STRAIGHT_SPEED, 30.0);
        // ... Turn left to approximately face beacon image...
        robot.turnOnSelf(robot.TURN_SPEED, Math.PI / 4);
        // ... go into it with robot vision guidance...
        robot.goToBeacon();
        // ... Finally, turn 90 degrees right to present sensors and pusher to the
        // beacon buttons.

        robot.leftFrontMotor.setPower(0);
        robot.leftRearMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
        robot.rightRearMotor.setPower(0);
    }
}
