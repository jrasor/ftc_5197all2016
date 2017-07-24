import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="GeometricSpeedControl", group="Robotics2")  // @Autonomous(...) is the other common choice
@Disabled
/*
    Opmode by Shane Beshlian

    Documentation goes here: what it does, and summary of gamepad inputs to get it
    done.
*/
public class SpeedControl extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    double motorSpeed = .8;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        leftMotor  = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");

        // Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            // Shane's speed control code is here.
            if(gamepad1.left_bumper)
            {
                if((motorSpeed / 2) >= 0.05 && motorSpeed != 1)
                    motorSpeed /= 2;
                else if(motorSpeed == 1)
                    motorSpeed -= 2;
            }
            if(gamepad1.right_bumper) {
                if ((motorSpeed * 2) <= 0.8)
                    motorSpeed *= 2;
                else if (motorSpeed == 0.8)
                    motorSpeed += 2;
            }
            // End of Shane's speed control code.

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            leftMotor.setPower(-gamepad1.left_stick_y * motorSpeed);
            rightMotor.setPower(-gamepad1.right_stick_y * motorSpeed);

            // idle() was made unnecessary in SDK v 2.35.
        }
    }
}