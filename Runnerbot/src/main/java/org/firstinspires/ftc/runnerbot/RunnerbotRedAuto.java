package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Runnerbot Red Auto", group="Runnerbot")
// @Autonomous(...) is the other common choice.
/*
    Opmode to drive the Tile Runner with 2 sticks of the gamepad. And control the sweeper.

    Shane Beshlian V. 0.1
*/

public class RunnerbotRedAuto extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private double sweepSpeed = 1.00;
    private int sweepState = 0; // 0 off, 1 shoot, 2 suck
    private boolean sweepToggle = false;
    private int sweepToggleState = 0;
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    DcMotor sweepMotor = null;
    Servo buttonPush = null;
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
        sweepMotor = hardwareMap.dcMotor.get("sweepMotor");
        buttonPush = hardwareMap.servo.get("buttonPush");
        // Set the drive motor directions:
        // Motors are mounted back to back, so they must run in opposite directions to have their
        // wheels turn in the same direction.
        leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set AndyMark motor to REVERSE
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Tempo: " + runtime.toString());

            // Run wheels in tank mode
            leftMotor.setPower(gamepad1.right_stick_y);
            rightMotor.setPower(gamepad1.left_stick_y);

            // Get Sweeper State
            if(gamepad1.right_bumper)
            {
                sweepToggleState = 1;
                sweepToggle = true;
            }
            else if(gamepad1.left_bumper)
            {
                sweepToggleState = 2;
                sweepToggle = true;
            }

            if(gamepad1.b)
            {
                sweepToggle = false;
            }
            if(gamepad1.dpad_left)
            {
                buttonPush.setPosition(1);
            }
            if(gamepad1.dpad_right)
            {
                buttonPush.setPosition(0);
            }
            if(sweepToggle)
            {
                switch (sweepToggleState) {
                    case 0:
                        sweepMotor.setPower(0);
                        break;
                    case 1:
                        sweepMotor.setPower(sweepSpeed);
                        break;
                    case 2:
                        sweepMotor.setPower(-sweepSpeed / 4);
                        break;
                }
            }
            else
            {
                sweepMotor.setPower(0);
            }

            telemetry.update();
        }
    }
}
