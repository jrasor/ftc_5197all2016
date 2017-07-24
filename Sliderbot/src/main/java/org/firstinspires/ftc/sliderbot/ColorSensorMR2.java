package org.firstinspires.ftc.sliderbot;

/* Copyright (c) 2016 Don Bosco Tech FTC Team 5197 "the GearHeads".

All rights reserved.

Created 9/29/2016 by Colton Mehlhoff of Modern Robotics using FTC SDK 2.2
Reuse permitted with credit where credit is due

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Don Bosco Tech nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
Modern Robotics Color Sensor Example with color number

Adapted for GearHead hardware from
http://modernroboticsinc.com/Content/Images/uploaded/AndroidStudioClass/MRI_Color_Sensor.java.

Configuration:
I2CDevice "color" (MRI Color Sensor with default I2C address 0x3c (0x1e 7-bit)

ModernRoboticsI2cColorSensor class is not being used because it can not access color number.
ColorSensor class is not being used because it can not access color number.

To change color sensor I2C Addresses, go to http://modernroboticsedu.com/mod/lesson/view.php?id=96
Support is available by emailing support@modernroboticsinc.com.
*/


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "I2C Color Sensor 2", group = "I2C Color Sensor") // @Autonomous(...) is the other common choice
//@Disabled
public class ColorSensorMR2 extends OpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    byte[] colorCcache;

    I2cDevice colorC;
    I2cDeviceSynch colorCreader;
    boolean bCurrState = false;
    boolean bPrevState = false;
    boolean LEDState = false;     //Tracks the mode of the color sensor; Active = true, Passive = false

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        //the below lines set up the configuration file
        colorC = hardwareMap.i2cDevice.get("color");
        colorCreader = new I2cDeviceSynchImpl(colorC, I2cAddr.create8bit(0x3c), false);
        colorCreader.engage();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() { // empty
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();

        //Active - For measuring reflected light. Cancels out ambient light
        //Passive - For measuring ambient light, eg. the FTC Color Beacon
        if(LEDState){   //Set the mode of the color sensor using LEDState
            colorCreader.write8(3, 0);
        }
        else{
            colorCreader.write8(3, 1);
        }
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Sensing: " + runtime.toString());
        // Toggle Active/Passive mode with gamepad1 X button
        if (!bCurrState && gamepad1.x) {  //If the X button is just now being pressed (was not pressed last time through the loop but now is)
            bCurrState = true;          //Change button state to true because the button has been pressed
            LEDState = !LEDState;       // Toggle LEDState
            if(LEDState){               //Toggle LED
                colorCreader.write8(3, 0);
            }
            else{
                colorCreader.write8(3, 1);
            }

        }
        if (!gamepad1.x) {                //If the X button is now pressed
            bCurrState = false;                  //Set the button state to false to indicate that it was released
        }

        colorCcache = colorCreader.read(0x04, 1);

        //display values
        telemetry.addData("2 #C", colorCcache[0] & 0xFF);
        telemetry.addData("LED", (LEDState ? "On" : "Off"));
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() { // empty
    }

}