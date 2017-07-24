package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
- Software team Shane Beshlian, James Villagrana, Anthony Duarte, Sebastian Jurado
- Version 0.1 11/29/16  JMR changed opmode name. Can't have two opmodes with same name.
*/

@Autonomous(name="Rangerbot Scores by Software Group", group="Rangerbot")
//@Disabled
public class RangerbotAutoComp extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareRangerbot robot = new HardwareRangerbot();   // Use a Rangerbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double LONG_TIME = 1.0e10;             // avoid timeout on encoderDrive
    static final double COUNTS_PER_MOTOR_REV = 1120.0;  // use 1440 for TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // 2.0 fr Pushbot, < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 2.9000; // determined by distance calibration.
    static final double RANGERBOT_DRIVEWHEEL_SEPARATION = 13.4115; // inches, by angle calibration.
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here.
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        idle();

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftMotor.getCurrentPosition(), // should be zero
                robot.rightMotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // jobDistanceCalibrate(60);             // run forward 60 inches
        // jobAngleCalibrate(10* Math.PI); // 5 full turns
        jobRunScript();
    }

    //    Run some inches, and measure physical distance traveled.
    public void jobDistanceCalibrate (double inches)
    {
        straightDrive(DRIVE_SPEED, inches);
    }

    //    Turn through some angle, and measure physical change in orientation.
    public void jobAngleCalibrate (double angle)
    {
        turnDrive(DRIVE_SPEED, angle);
    }

    // Step through each leg of the path,
    // Note: Reverse movement is obtained by setting a negative distance (not speed)
    public void jobRunScript(){
        straightDrive(DRIVE_SPEED, 12);
        turnDrive(TURN_SPEED, 2*Math.PI);
        straightDrive(DRIVE_SPEED, -12);
    }

    /*
     *  Method to perform an absolute move, based on encoder counts.
     *  Encoders are reset at the beginning of the move, ignoring the current position.
     *  Move will stop if any of three conditions occur:
     *  1) One motor gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            newLeftTarget = (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = (int) (rightInches * COUNTS_PER_INCH);
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));

            // While we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
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

    //    Wrapper for encoderDrive
    public void straightDrive(double speed, double inches) {
        encoderDrive(speed, inches, inches, LONG_TIME);
    }

    //    Wrapper for encoderDrive. Turns about axis halfway between drive wheels.
    public void turnDrive(double speed, double radians) {
        int turnTarget;

        turnTarget = (int)(radians * RANGERBOT_DRIVEWHEEL_SEPARATION / 2);
        encoderDrive(speed, -turnTarget, turnTarget, LONG_TIME);
    }
}
