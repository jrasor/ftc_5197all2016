package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Runnerbot, a developed AndyMark Tile Runner.
 *
 * This hardware class assumes motor, sensor and servo names have been configured
 * in the robot configuration file.
 */

public class Runnerbot
{
    /* Public OpMode members. */
    public static final double COUNTS_PER_MOTOR_REV = 1120.0; // use 1440 for TETRIX Motor Encoder
    public static final double DRIVE_GEAR_REDUCTION = 1.0;    // 2.0 for Pushbot, < 1.0 if geared UP
    public static final double WHEEL_DIAMETER_INCHES = 4.04;   // calibrated
    public static final double DRIVEWHEEL_SEPARATION = 15.4;  // calibrated inches
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    public static final double TURN_SPEED = 0.5;
    public static final double STRAIGHT_SPEED = 0.6;
    public double sweepSpeed = 1.00;
    public boolean sweepToggle = false;
    public int sweepToggleState = 0;
    public DcMotor  leftMotor  = null;
    public DcMotor  rightMotor = null;
    public DcMotor  sweepMotor = null;
    public Servo buttonPush = null; // beacon button pusher
    public ColorSensor color = null;
    public I2cDevice range = null;
    private LinearOpMode currentOpMode;

    /* local robot members. */
    HardwareMap hwMap       =  null;

    /* Constructor */
    public Runnerbot(LinearOpMode linearOpMode){
        currentOpMode = linearOpMode;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotor  = hwMap.dcMotor.get("leftMotor");
        rightMotor = hwMap.dcMotor.get("rightMotor");
        sweepMotor = hwMap.dcMotor.get("sweepMotor");
        leftMotor.setDirection(DcMotor.Direction.REVERSE); // Tetrix motor: FORWARD
        rightMotor.setDirection(DcMotor.Direction.FORWARD);// Tetrix motor: REVERSE
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        sweepMotor.setPower(0);

        // May want to use RUN_USING_ENCODERS if encoders are installed.
        // Generally, Autonomous modes will run encoded, and Driver (TeleOp) modes
        //   will run unencoded.
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set up servos and sensors
        // ** Move these to init method for subclass, and let this Runnerbot class
        // **   deal only with the drive train.
        buttonPush = hwMap.servo.get("buttonPush");
        // ...
    }

    public void initEncodedDrive(){
        // Zero the encoder counts targets. Side effect is to remove
        //   power, but we will do that explicitly.
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    /*
     *  Do a relative move, based on encoder counts.
     *  Move will stop if any of two conditions occur:
     *  1) One or the other drive motor gets to the desired position
     *  2) Driver stops the opmode running.
    */
    public void encoderDrive(double leftSpeed, double rightSpeed,
                             double leftInches, double rightInches) {
        int newLeftTarget;
        int newRightTarget;

        // Determine new target position, and pass to motor controller
        newLeftTarget = leftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        newRightTarget = rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
        leftMotor.setTargetPosition(newLeftTarget);
        rightMotor.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Go!
        leftMotor.setPower(Math.abs(leftSpeed));
        rightMotor.setPower(Math.abs(rightSpeed));

        // keep looping while we are still active, and both motors are running.
        while ( leftMotor.isBusy() && rightMotor.isBusy() && currentOpMode.opModeIsActive()) {
            // just wait
        }
    }

    public void turnAngleRadiusDrive (double speed, double angle, double radius) {

        // One or both of these arcs could be negative.
        double leftRadius = radius - DRIVEWHEEL_SEPARATION/2.0;
        double leftArc  = leftRadius * angle;
        double rightRadius = radius + DRIVEWHEEL_SEPARATION/2.0;
        double rightArc = rightRadius * angle;
        // One or both of these wheel targets or speeds could be negative.
        // ** Need to clip and scale speeds here.
        double leftSpeed = speed * leftRadius/radius;
        double rightSpeed = speed * rightRadius/radius;

        encoderDrive(leftSpeed, rightSpeed, leftArc, rightArc);

        // Degenerate cases: angle = 0, R = 0, R = d/2, R = +infinity
        // (straight drive).
    }

    //    Wrapper for turnAngleRadius
    // ** make this a wrapper for encoderDrive instead.
    public void turnArcRadiusDrive(double speed, double angle, double radius) {
        double targetAngle = angle / radius;
        turnAngleRadiusDrive (speed, targetAngle, radius);
    }

    public void straightDrive (double speed, double inches) {
        // Wheels may be slightly different in diameter. Symptom: drives slightly
        // curved. Calibrate to determine this.
        encoderDrive(speed, speed, inches, inches);
    }
}

