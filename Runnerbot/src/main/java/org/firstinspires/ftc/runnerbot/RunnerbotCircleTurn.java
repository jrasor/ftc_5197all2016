package org.firstinspires.ftc.teamcode;

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
 *
 *
 */

@Autonomous(name="Runnerbot: Circular Turn", group="Rangerbot")
//@Disabled
public class RunnerbotCircleTurn extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareRunnerbot robot = new HardwareRunnerbot();  // Use a Runnerbot's hardware


    static final double LONG_TIME = 1.0e10;             // avoid timeout on encoderDrive
    static final double COUNTS_PER_MOTOR_REV = 1120.0;  // use 1440 for TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // 2.0 for Pushbot, < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.9000; // determined by distance calibration.
    static final double RANGERBOT_DRIVEWHEEL_SEPARATION = 14.4; // inches, by angle calibration.
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double TURN_SPEED = 0.5;
    private double pow = 0.5;

    private ElapsedTime runtime = new ElapsedTime();

    byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable

    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    public static final int RANGE1_REG_START = 0x04; //Register to start reading
    public static final int RANGE1_READ_LENGTH = 2; //Number of byte to read

    public I2cDevice RANGE1;
    public I2cDeviceSynch RANGE1Reader;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware are class does all the work here.
         */
        robot.init(hardwareMap);


        RANGE1 = hardwareMap.i2cDevice.get("range");
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        idle();

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Curved Path", "Starting at %7d :%7d",
                robot.leftMotor.getCurrentPosition(), // should be zero
                robot.rightMotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // jobDistanceCalibrate(60.0, 60.0);  // run 60" arc length, 60" radius
        // jobAngleCalibrate(10* Math.PI, RANGERBOT_DRIVEWHEEL_SEPARATION); // 5 full turns
        jobRunScript();
    }

    //    Run some inches, and measure physical distance traveled.
    /*public void jobDistanceCalibrate (double inches)
    {
        straightDrive(DRIVE_SPEED, inches);
    }
    */
    //    Turn through some angle, and measure physical change in orientation.
    /*public void jobAngleCalibrate (double angle)
    {
        turnDrive(DRIVE_SPEED, angle);
    }
    */
    // Step through each leg of the path,
    // Note: Reverse movement is obtained by setting a negative distance (not speed)
    public void jobRunScript(){
        //    turnArcRadiusDrive(TURN_SPEED, 10.0, 20.0); // Run 20" around a 20" radius arc
        //    turnAngleRadiusDrive(TURN_SPEED, 0.5, 20.0);// Turn 1/2 radian around a 20" radius arc

        // Go to Button area
        turnAngleRadiusDrive(TURN_SPEED, (73 * Math.PI / 180) , 86); // adapt to push cap ball, park

        // Turn 90 degrees
        turnAngleRadiusDrive(TURN_SPEED, Math.PI / 2, 0);



    }
    // Get Parallel
    public void getParallel()
    {
        range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);

        telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
        telemetry.addData("ODS", range1Cache[1] & 0xFF);
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
        while((range1Cache[1] & 0xFF) < 1)
        {
            turnAngleRadiusDrive(0.5, 5, 0);
        }
    }
    //    Adaptation of encoderDrive for different left and right speeds.
    public void turnAngleRadiusDrive (double speed, double angle, double radius) {
        double leftArc  = (radius - RANGERBOT_DRIVEWHEEL_SEPARATION/2.0) * angle;
        double rightArc = (radius + RANGERBOT_DRIVEWHEEL_SEPARATION/2.0) * angle;
        double leftSpeed = speed * (radius - RANGERBOT_DRIVEWHEEL_SEPARATION/2.0)/
                (radius + RANGERBOT_DRIVEWHEEL_SEPARATION);
        double rightSpeed = speed;

        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            newLeftTarget = (int) (leftArc * COUNTS_PER_INCH);
            newRightTarget = (int) (rightArc * COUNTS_PER_INCH);
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Adjust speeds, reset the timeout time.
            // Left motor speed (R â€“ d/2)/(R + d/2) * max speed.
            // Right motor goes arc = (R + d/2) * Î¸ at max speed.
            // might need to clip speeds here
            robot.leftMotor.setPower(Math.abs(leftSpeed));
            robot.rightMotor.setPower(Math.abs(rightSpeed));
            runtime.reset();
            // This should take care of Î¸ and R > 0. There are 3 other signed cases.
            // Degenerate cases: Î¸ = 0, R = 0, R = Â±d/2, R = âˆž.
            // While we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < LONG_TIME) &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Goal", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Now at", "%7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    //    Wrapper for turnAngleRadius
    public void turnArcRadiusDrive(double speed, double angle, double radius) {
        double targetAngle = angle / radius;
        turnAngleRadiusDrive (speed, targetAngle, radius);
    }
}
