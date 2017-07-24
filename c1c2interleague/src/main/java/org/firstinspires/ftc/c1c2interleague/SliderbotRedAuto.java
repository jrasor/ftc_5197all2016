package org.firstinspires.ftc.c1c2interleague;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 *
 *  Shane 1/4/17 v. 0.1 go to Gears beacon, then turn right 90 degrees to present
 *    sensors and pusher to beacon buttons. WORKS 1/13 on Runnerbot.
 *    ** 1/13/17 Color sensor seems to malfunction.
 *  Shane 2/8/17 v 0.2 simplified. This uses Sliderbot, a mecanum wheel drive robot.
 *  Shane 2/9/17 v 0.3. Adds back the color sensor, and finishes final approach with port
 *    side of bot presented to the beacon. ** Approach is slow, taking almost the whole
 *    30 seconds of Autonomous. Color sensor not reporting anything useful.
 *  Shane 2/9/17 v 0.3.1 Fixed color sensor. WORKS to capture one beacon.
 *  Shane 2/9/17 v 0.4 Added path to approach second beacon. ** FAILS on second: still
 *    chasing the first, and gets lost.
 *  JMR 2/10/17 v 0.4.1 Corrected lost approach to second beacon by giving time for Vuforia
 *    to look.
 *  Shane 2/10/17 v 0.5 Tuned second initial and final beacon approaches. First capture
 *    success rate about 90%, second about 30%. This is near production code.
 */

@Autonomous(name="Sliderbot RED BEACONS AUTO", group="Interleague")
//@Disabled
public class SliderbotRedAuto extends LinearOpMode {

    /* Declare OpMode members. */
    //public VVField field = new VVField(this);
    public Sliderbot robot = new Sliderbot(this);
    private final double STRAIGHT_SPEED = 0.60;
    private final double TURN_SPEED = 0.60;
    private final double TIGHT_TURN_RATIO = 0.40; // Was 4.0
    private final double WIDE_TURN_RATIO = 0.60;
    private final double SEARCH_SPEED = 0.30;
    // Give time for Vuforia to stare at image and identify it.
    private final int IMAGE_ID_TIME = 700;
    // Give time for pusher to hold beacon button.
    private final int BUTTON_HOLD_TIME = 750; // Hold the beacon button long enough to capture it.
    String imageName = "nothing";

    byte[] colorCcache;

    I2cDevice colorC;
    I2cDeviceSynch colorCreader;
    boolean bCurrState = false;
    boolean bPrevState = false;
    boolean LEDState = false;

    boolean seeColor = false;
    @Override
    public void runOpMode() {
        robot.initEncodedDrive(hardwareMap);
        robot.initBeaconPusher();
        //  Get an initialized VuforiaLocalizer, and trackable images.
        VuforiaLocalizer vuforia = robot.initVuforia();
        robot.allTrackables = robot.initBeaconImages(vuforia);

        //  Prepare the color sensor.
        colorC = hardwareMap.i2cDevice.get("color");
        colorCreader = new I2cDeviceSynchImpl(colorC, I2cAddr.create8bit(0x3c), false);
        colorCreader.engage();
        colorCreader.write8(3, 1); // PASSIVE mode. Look at the beacon's own light, not
          // the light reflected from it by a lit LED.

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status: ", "Ready!");
        telemetry.update();
        waitForStart();

        jobRunScript();
    }

    // Step through each leg of the path,
    // Note: Reverse movement is obtained by setting a negative distance (not speed)
    public void jobRunScript(){
        // Place robot's left edge 4' from left corner, or exactly two tiles.
        // robot.buttonPush.setPosition(1); // ** Already done in initBeaconPusher.
        // Run straight forward...
        robot.driveStraight(1.0, 16.0);
        // ... Turn left to approximately face beacon image Gears ...
        robot.turnLeft (.95, TIGHT_TURN_RATIO, 70.0);
        // ... go into it with robot vision guidance...
        robot.initEncodedDrive(hardwareMap);
        robot.goToBeacon();
        // ... turn 90 degrees left to present sensors and pusher to the
        // beacon buttons...

        robot.turnAngle(Sliderbot.TURN_SPEED, Math.PI/2);
        // ... Finally, back up and move in to present sensor to right half of the beacon.
        //   We will search from there with the color sensor.
        robot.driveStraight(Sliderbot.STRAIGHT_SPEED/4.0, -4.5);
        robot.driveSideways(Sliderbot.STRAIGHT_SPEED/4.0, 1.3); // ** This invites trouble.
        // ** We can eliminate it with a different target X.
        while(!seeColor) {
            colorCcache = colorCreader.read(0x04, 1);
            telemetry.addData("2 #C", colorCcache[0] & 0xFF);
            telemetry.addData("LED", (LEDState ? "On" : "Off"));
            telemetry.update();
            if ((colorCcache[0] & 0xFF) == 10) // 10 for RED, 3 for Blue
            {
                robot.buttonPush.setPosition(0);
                seeColor = true;
                break;
            } else {
                robot.driveStraight(0.4, 1.0);
            }
        }
        sleep(BUTTON_HOLD_TIME); // This WORKS very well.
        robot.buttonPush.setPosition(1);

        // Now we get in position to observe second beacon image, Tools.
        robot.turnLeft(.95, WIDE_TURN_RATIO, -70.0);
        robot.turnAngle(Sliderbot.TURN_SPEED, -Math.PI/4);
        sleep(IMAGE_ID_TIME);
        robot.initEncodedDrive(hardwareMap);
        robot.goToBeacon();
        // beacon buttons...
        robot.turnAngle(Sliderbot.TURN_SPEED, Math.PI/2);
        // ... Finally, back up and move in to present sensor to right half of the beacon.
        //   We will search from there with the color sensor.
        //robot.turnAngle(Sliderbot.TURN_SPEED, -Math.PI / 48);
        robot.driveStraight(Sliderbot.STRAIGHT_SPEED/4.0, -3.5);
        robot.driveSideways(Sliderbot.STRAIGHT_SPEED/4.0, 1.3); // ** This invites trouble.
        // ** We can eliminate it with a different target X.

        seeColor = false;
        while(!seeColor) {
            colorCcache = colorCreader.read(0x04, 1);

            //display values
            telemetry.addData("2 #C", colorCcache[0] & 0xFF);
            telemetry.addData("LED", (LEDState ? "On" : "Off"));
            telemetry.update();
            if ((colorCcache[0] & 0xFF) == 10) // 10 for RED, 3 for Blue
            {
                robot.buttonPush.setPosition(0);    
                seeColor = true;
                break;
            } else {
                robot.driveStraight(0.4, 1.0);
            }
        }
    }
}
