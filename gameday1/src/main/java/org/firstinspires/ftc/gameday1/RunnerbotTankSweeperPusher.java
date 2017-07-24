package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Sweeper Pusher", group="Meet 1")
//@Disabled
/*
    Opmode to drive the Tile Runner in tank mode, operate sweeper and beacon pusher.

    Shane Beshlian V. 0.1
    JMR 12/22/16 v. 0.2 Uses expanded Runnerbot class.
*/

public class RunnerbotTankSweeperPusher extends LinearOpMode {

    /* Declare OpMode members. */
    Runnerbot robot = new Runnerbot(this);
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Time: ", runtime.toString());

            // Run wheels in tank mode
            robot.leftMotor.setPower(-gamepad1.left_stick_y);
            robot.rightMotor.setPower(-gamepad1.right_stick_y);

            // Operate the sweeper
            if(gamepad1.right_bumper)
            {
                robot.sweepToggleState = 1;
                robot.sweepToggle = true;
            }
            else if(gamepad1.left_bumper)
            {
                robot.sweepToggleState = 2;
                robot.sweepToggle = true;
            }

            if(gamepad1.b)
            {
                robot.sweepToggle = false;
            }

            if(robot.sweepToggle)
            {
                switch (robot.sweepToggleState) {
                    case 0: // off
                        robot.sweepMotor.setPower(0);
                        break;
                    case 1: // shoot
                        robot.sweepMotor.setPower(robot.sweepSpeed);
                        break;
                    case 2: // suck
                        robot.sweepMotor.setPower(-robot.sweepSpeed / 4);
                        break;
                }
            }
            else
            {
                robot.sweepMotor.setPower(0);
            }

            // Operate the beacon button pusher
            if(gamepad1.dpad_left)
            {
                robot.buttonPush.setPosition(1);
            }
            if(gamepad1.dpad_right)
            {
                robot.buttonPush.setPosition(0);
            }
            telemetry.update();
        }
    }
}
