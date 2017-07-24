package org.firstinspires.ftc.development;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Drive the Runnerbot", group="Runnerbot")
//@Disabled
/*
    Opmode to drive the Tile Runner in tank mode.
    It does not operate sweeper or beacon pusher.

    JMR 1/9/17  v. 0.1 Uses expanded Runnerbot class.
*/

public class RunnerbotTank extends LinearOpMode {

    /* Declare OpMode members. */
    public VVField field = new VVField(this);
    public Runnerbot robot = new Runnerbot(this);
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.initDrive(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Time: ", runtime.toString());

            //  Run wheels in tank mode
            //  Temper joystick response
            double l = gamepad1.left_stick_y;
            double r = gamepad1.right_stick_y;
            double exp = 3.8;
            double p;
            if (l >= 0) {p = -Math.pow(l, exp);} else { p = Math.pow(-l, exp);}
            robot.leftMotor.setPower(p);
            if (r >= 0) {p = -Math.pow(r, exp);} else { p = Math.pow(-r, exp);}
            robot.rightMotor.setPower(p);

            telemetry.update();
        }
    }
}
