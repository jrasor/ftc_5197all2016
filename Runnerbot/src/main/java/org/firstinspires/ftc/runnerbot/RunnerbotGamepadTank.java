package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Runnerbot Gamepad Tank Mode", group="Runnerbot")  // @Autonomous(...) is the other common choice
/*
    Opmode to drive the robot with only the left stick of the gamepad.

    Bro. John version 0.1, Tile Runner hardware. Clipping illegal motor power is not implemented.

*/

public class RunnerbotGamepadTank extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftMotor1 = null;
    DcMotor leftMotor2 = null;
    DcMotor rightMotor1 = null;
    DcMotor rightMotor2 = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Stato", "Iniziato");
        telemetry.update();

        /* Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        leftMotor1  = hardwareMap.dcMotor.get("leftMotor1");
        leftMotor2  = hardwareMap.dcMotor.get("leftMotor2");
        rightMotor1 = hardwareMap.dcMotor.get("rightMotor1");
        rightMotor2 = hardwareMap.dcMotor.get("rightMotor2");

        // Set the drive motor directions:
        // Motors are mounted back to back, so they must run in opposite directions to have their
        // wheels turn in the same direction.
        leftMotor1.setDirection(DcMotor.Direction.FORWARD); // Set AndyMark motor to REVERSE
        leftMotor2.setDirection(DcMotor.Direction.FORWARD);
        rightMotor1.setDirection(DcMotor.Direction.REVERSE);
        rightMotor2.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Tempo: " + runtime.toString());

            // eg: Run wheels in tank mode
            leftMotor1.setPower (gamepad1.left_stick_y);
            leftMotor2.setPower (gamepad1.left_stick_y);
            rightMotor1.setPower(gamepad1.right_stick_y);
            rightMotor2.setPower(gamepad1.right_stick_y);
            telemetry.update();
        }
    }
}
