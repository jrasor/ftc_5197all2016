/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.sliderbot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 */

@Autonomous(name="Calibrate", group="Testing")
//@Disabled
public class Calibrate extends LinearOpMode {

    /* Declare OpMode members. */
//    public VVField field = new VVField(this);
    public Sliderbot robot = new Sliderbot(this);
    static final double     DRIVE_SPEED             = 0.50;
    static final double     TURN_SPEED              = 0.50;
    static final double     WIDE_TURN_RATIO         = 0.80;
    static final double     TIGHT_TURN_RATIO        = 0.40;

    @Override
    public void runOpMode() {
        robot.initUnencodedDrive(hardwareMap);
        idle();
        waitForStart(); // Driver presses PLAY button

        // Step through each leg of the path.
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        //robot.initEncodedDrive(hardwareMap);
        //   Test straight drives and turns.
        //robot.driveStraight(DRIVE_SPEED, 48.0);  // Forward
        //robot.driveStraight(DRIVE_SPEED, -48.0);  // Back
        robot.driveSideways( DRIVE_SPEED, -36.0);    // Left 3 feet
        robot.driveSideways( DRIVE_SPEED, 36.0);    // Right 3 feet
        //robot.driveSideways(DRIVE_SPEED, 12.0);    // Right a foot

        //robot.turnOnSelf(Sliderbot.TURN_SPEED, Math.PI/4); // right 45 degrees.
        //robot.turnAngle(Sliderbot.TURN_SPEED, Math.PI/2); // CW 90 degrees. FEW DEG SHORT.
        //   Test 4 turn cases.
        //robot.turnLeft(DRIVE_SPEED, TURN_RATIO, 24);
        //robot.turnRight(DRIVE_SPEED, TIGHT_TURN_RATIO, 24);
    }
}
