package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Look Ma One Hand", group="Robotics1")  // @Autonomous(...) is the other common choice
/*
    Opmode to drive the robot with only the left stick of the gamepad.

    Bro. John version 0.5. Clipping illegal motor power is not implemented.

    Stick forward: drive forward.
    Stick back: drive backward.
    Stick left: steer to left.
    Stick right: steer to right.
*/

public class RangerBot1Stick extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;

    private double sumPower = 0.0;
    private double diffPower = 0.0;
    private double leftMotorPower = 0.0;
    private double rightMotorPower = 0.0;
    private double maxPower = 0.0;
    private final double MAX_POWER = 1.0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        leftMotor  = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");

        // Set the drive motor directions:
        // Motors are mounted back to back, so they must run in opposite directions to have their
        // wheels turn in the same direction.
        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Pushbot motor is geared, so FORWARD
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set AndyMark motor to FORWARD

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            // calculate sum and difference of motor powers.
            sumPower = -gamepad1.left_stick_y; // forward stick is negative; we want
            // power forward.
            diffPower = gamepad1.left_stick_x; // left-right stick steers by running motors
            // differently.

            // calculate motor power as a linear combination of sum and difference.
            leftMotorPower = (sumPower + diffPower) / 2;
            rightMotorPower = (sumPower - diffPower) / 2;

            // clip illegal power levels.
            maxPower = Math.max(Math.abs(leftMotorPower), Math.abs(rightMotorPower));
            maxPower = Math.min(maxPower, MAX_POWER);
            //leftMotorPower /= maxPower;
            //rightMotorPower /= maxPower;

            // eg: Run wheels in tank mode
            leftMotor.setPower(leftMotorPower);
            rightMotor.setPower(rightMotorPower);

            // Display power applied to each motor
            telemetry.addData ("Left Motor power:  ", "%5.2f", leftMotorPower);
            telemetry.addData ("Right motor power: ", "%5.2f", rightMotorPower);
            telemetry.update();
        }
    }
}
