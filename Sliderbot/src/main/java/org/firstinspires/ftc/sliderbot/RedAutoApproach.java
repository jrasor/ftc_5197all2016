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

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 * Autonomous mode for Red team to approach nearest beacon.
 *
 * Version history:
 * 2/8/17 v 0.1 JMR: just does a few moves to get in position for Vuforia assisted
 *   beacon approach. ** Fails to do Vuforia-assisted beacon approach.
 */

@Autonomous(name="Red Auto Approach", group="Testing")
//@Disabled
public class RedAutoApproach extends LinearOpMode {

    public Sliderbot robot = new Sliderbot(this);
    static final double     DRIVE_SPEED             = 0.4;
    static final double     TURN_SPEED              = 0.4;
    static final double     TURN_RATIO              = 0.4;

    @Override
    public void runOpMode() {
        robot.initEncodedDrive(hardwareMap);
        VuforiaLocalizer vuforia = robot.initVuforia();
        robot.allTrackables = robot.initBeaconImages(vuforia);
        robot.initBeaconPusher();
        waitForStart(); // Driver presses PLAY button

        // Step through each leg of the path.
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        //   Test straight drives and turns.
        robot.driveStraight(DRIVE_SPEED, 16.0);
        robot.turnLeft(TURN_SPEED, TURN_RATIO, 70.0);
        robot.initEncodedDrive(hardwareMap);
        robot.goToBeacon(); // ** Image acquired, position and heading good, but motors just buzz.
    }
}
