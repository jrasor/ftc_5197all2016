package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Gamepad Tank Mode", group="Robotics1")  // @Autonomous(...) is the other common choice
/*
    Opmode to drive the robot with only the left stick of the gamepad.

    Bro. John version 0.5. Clipping illegal motor power is not implemented.

    Stick forward: drive forward.
    Stick back: drive backward.
    Stick left: steer to left.
    Stick right: steer to right.
*/

public class RangerbotGamepadTank extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;

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

            // eg: Run wheels in tank mode
            leftMotor.setPower(gamepad1.right_stick_y);
            rightMotor.setPower(gamepad1.left_stick_y);
            telemetry.update();
        }
    }
}
