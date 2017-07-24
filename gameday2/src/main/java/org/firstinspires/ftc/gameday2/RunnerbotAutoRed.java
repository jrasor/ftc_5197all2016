package org.firstinspires.ftc.gameday2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

/**

 *  Shane Beshlian -- Adapting OpMode to work for our autonomous plan.
 *  V. 0.1 - Doesnt Quite work right, goes into an infinite loop.
 *  JMR 1/13/17 v. 0.2 Pushes Red Cap Ball off Center Vortex base. Works.
 *  JMR 1/13/17 v. 0.3 Uses Runnerbot class from vision project. Calibration
 *    WAY off, but WORKS.
 */

@Autonomous(name="Red Auto Push Park", group="Meet 2")
//@Disabled
public class RunnerbotAutoRed extends LinearOpMode {

    /* Declare OpMode members. */
    Runnerbot robot = new Runnerbot(this);

    @Override
    public void runOpMode() {
        robot.initDrive(hardwareMap);
        robot.initEncodedDrive();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        jobRunScript();
    }

    // Step through each leg of the path,
    // Note: Reverse movement is obtained by setting a negative distance (not speed)
    public void jobRunScript(){
        // Place robot's right edge 4' from right corner, or exactly two tiles.
        // Run forward, turning left toward beacon area, pushing Red Cap Ball
        // off Center Vortex and partially parking there.
        robot.turnAngleRadiusDrive(robot.TURN_SPEED, (65 * Math.PI / 180), 45.0);
        // ** To do:
        // Continue past Center Vortex, turning into the beacon area. This should
        //   put the robot within a couple feet of the "Gears" beacon image, close
        //   enough for Vuforia to pick it up.
        // Turn some degrees CCW, pointing the portside color sensor to the "Gears" beacon
        // ** sense beacon state, adjust position, push button somewhere in here
        //robot.turnAngleRadiusDrive(robot.TURN_SPEED, 120*Math.PI / 180, 0); // ** not enough

        // ** Continue with getParallel ...
    }
    // Get Parallel

}
