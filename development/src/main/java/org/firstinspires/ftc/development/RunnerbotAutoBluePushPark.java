package org.firstinspires.ftc.development;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

/**

 *  Shane Beshlian -- Adapting OpMode to work for our autonomous plan.
 *  V. 0.1 - Doesnt Quite work right, goes into an infinite loop.
 *  JMR 12/22/16 v. 0.2 Pushes Blue Cap Ball off Center Vortex base, then
 *    continues to near the "Wheels" beacon. It's too far from there to sense
 *    beacon state, or push a button.
 *  JMR 12/22/16 v. 0.3 Uses robot methods to drive script segments, instead
 *    of using opmode methods to do that.
 */

@Autonomous(name="Runnerbot Blue Auto PushPark", group="Runnerbot")
//@Disabled
public class RunnerbotAutoBluePushPark extends LinearOpMode {

    /* Declare OpMode members. */
    public VVField field = new VVField(this);
    public Runnerbot robot = new Runnerbot(this);

    // Ultrasonic range sensor properties
    byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable
    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    public static final int RANGE1_REG_START = 0x04; //Register to start reading
    public static final int RANGE1_READ_LENGTH = 2; //Number of byte to read
    public I2cDevice RANGE1;
    public I2cDeviceSynch RANGE1Reader;

    @Override
    public void runOpMode() {
        robot.initDrive(hardwareMap);

        // Initialize the range sensor
        RANGE1 = hardwareMap.i2cDevice.get("range");
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();

        //  Robot initialization has set this not to use encoders.
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        jobRunScript();
    }

    // ** Design and document correct robot starting location.
    public void jobRunScript(){
        // Place robot's left edge 4' from left corner, or exactly two tiles.
        // Run forward, turning right toward beacon area, pushing Blue Cap Ball
        // off Center Vortex and partially parking there.
        robot.turnAngleRadiusDrive(robot.TURN_SPEED, (-45 * Math.PI / 180), -62.0);
        // ** To do:
        // Continue past Center Vortex, turning into the beacon area. This should
        //   put the robot within a couple feet of the "Wheels" beacon image, close
        //   enough for Vuforia to pick it up.
        // Turn some degrees CW, pointing the portside color sensor to the "Wheels" beacon
        // ** sense beacon state, adjust position, push button somewhere in here
        //robot.turnAngleRadiusDrive(robot.TURN_SPEED, -120*Math.PI / 180, 0); // ** not enough

        // ** Continue with getParallel ...
    }
    // Get Parallel
    public void getParallel()
    {
        range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);

        telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
        telemetry.addData("ODS", range1Cache[1] & 0xFF);
        telemetry.update();
        while((range1Cache[1] & 0xFF) < 1)
        {
            robot.turnAngleRadiusDrive(0.5, 5, 0);
        }
    }
}
