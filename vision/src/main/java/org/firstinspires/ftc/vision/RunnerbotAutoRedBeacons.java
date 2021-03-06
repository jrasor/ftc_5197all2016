package org.firstinspires.ftc.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 *
 *  JMR 1/4/17 v. 0.1 go to Gears beacon, then turn right 90 degrees to present
 *    sensors and pusher to beacon buttons.
 *    ** 1/13/17 Color sensor seems to malfunction.
 */

@Autonomous(name="Runnerbot Red Auto Beacons", group="Runnerbot")
//@Disabled
public class RunnerbotAutoRedBeacons extends LinearOpMode {

    /* Declare OpMode members. */
    //public VVField field = new VVField(this);
    public Runnerbot robot = new Runnerbot(this);


    byte[] colorCcache;

    I2cDevice colorC;
    I2cDeviceSynch colorCreader;
    boolean bCurrState = false;
    boolean bPrevState = false;
    boolean LEDState = false;     //Tracks the mode of the color sensor; Active = true, Passive = false
    boolean seeColor = false;

    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    Servo servo = null;
    @Override
    public void runOpMode() {
        robot.initDrive(hardwareMap);

        telemetry.addData("Status", "Initialized");

        //the below lines set up the configuration file
        servo = hardwareMap.servo.get("Servo1");

        // Set up Color Sensor
        colorC = hardwareMap.i2cDevice.get("color");
        colorCreader = new I2cDeviceSynchImpl(colorC, I2cAddr.create8bit(0x3c), false);
        colorCreader.engage();

        //  Robot initialization has set this not to use encoders.
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //  Get an initialized VuforiaLocalizer.
        VuforiaLocalizer vuforia = robot.initVuforia();

        //  Get the beacon images trackable and located.
        robot.allTrackables = robot.initBeaconImages(vuforia);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Color Sensor ACTIVE or PASSIVE

        if(LEDState)
        {   //Set the mode of the color sensor using LEDState
            colorCreader.write8(3, 0);
        }
        else
        {
            colorCreader.write8(3, 1);
        }

        jobRunScript();
    }

    // Step through each leg of the path,
    // Note: Reverse movement is obtained by setting a negative distance (not speed)
    public void jobRunScript(){
        robot.initEncodedDrive();
        // Place robot's left edge 4' from left corner, or exactly two tiles.
        // Run straight forward...
        robot.straightDrive(.4, 20.0);
        // ... Turn left to approximately face beacon image...
        robot.turnAngleRadiusDrive(.4, (Math.PI / 2), 24.0);
        // ... go into it with Vuforia vision guidance...
        robot.initEncodedDrive();
        robot.goToBeacon();
        // ... Finally, turn 90 degrees right to present sensors and pusher to the
        // beacon buttons.
        robot.turnOnSelf(0.1, Math.PI/4);

        // ** Continue Autonomous beacon capture plan here.
        while(!seeColor) {
            colorCcache = colorCreader.read(0x04, 1);

            //display values
            telemetry.addData("2 #C", colorCcache[0] & 0xFF);
            telemetry.addData("LED", (LEDState ? "On" : "Off"));
            telemetry.update();
            if ((colorCcache[0] & 0xFF) == 10)
            {
                seeColor = true;
                break;
            }
            else
            {
                robot.straightDrive(robot.STRAIGHT_SPEED,1.0);
            }
        }
        servo.setPosition(MAX_POS);
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }
}
