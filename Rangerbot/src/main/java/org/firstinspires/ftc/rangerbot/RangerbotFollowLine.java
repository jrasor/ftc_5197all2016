package org.firstinspires.ftc.teamcode;

/*
Copyright (c) 2016 Intelitek http://ftc.edu.intelitek.com

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Intelitek nor the names of their contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Copyright (c) 2016 Don Bosco Technical Institute Robotics, FTC Team 5197

This is modified source redistribution of Intelitek's work, and is distributed
under the same conditions as in his copyright notice above.
*/

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/*****************************************************************************************
 Uses data from optical distance sensor to follow beige masking tape line on Velocity
 Vortex arena.

 Bro. John
 11/11/16 version 0.1 crude, but works. Can't follow sharp turns, and can't recover
 from losing the line. It can just barely follow a masking tape line on the floor
 outside the arena.

 No gamepad input necessary, since this is an Autonomous opmode. Just put the robot
 with drive wheels front, sensor over the masking tape line in any orientation,
 run the opmode, sit back and watch the show.
 ****************************************************************************************/

@Autonomous(name = "Rangerbot Follow Line", group = "Sensor")
//@Disabled
public class RangerbotFollowLine extends LinearOpMode {
    HardwareRangerbot robot = new HardwareRangerbot();
    DcMotor leftMotor;
    DcMotor rightMotor;
    OpticalDistanceSensor odsSensor;  // Hardware Device Object
    static final double RUN_POWER = 0.2;
    static final double FLOOR_REFLECTANCE = 0.2;
    static final double LINE_REFLECTANCE = 0.55;
    static final double THRESHOLD_REFLECTANCE = (LINE_REFLECTANCE + FLOOR_REFLECTANCE)/2;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        odsSensor = hardwareMap.opticalDistanceSensor.get("ods");
        double reflectance = 0.0;

        waitForStart();

        // While the op mode is active, loop: read light, decide direction to steer.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
            // send the info back to driver station using telemetry function.
            // Write the reflectance detected to a variable
            reflectance = odsSensor.getLightDetected();

            // If the sensor is on the line
            // only the right motor rotates to move it off the line
            if (reflectance >= THRESHOLD_REFLECTANCE) {
                rightMotor.setPower(RUN_POWER);
                leftMotor.setPower(0);
            }
            // Otherwise (if the sensor is off the line)
            // only the left motor rotates to move it back toward the line
            else {
                leftMotor.setPower(RUN_POWER);
                rightMotor.setPower(0);
                telemetry.addData("Reflectance", "%7f4", reflectance);
                telemetry.update();
            }
        }
    }
}
