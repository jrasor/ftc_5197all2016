/* Copyright (c) ... */
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
/*****************************************************************************************
 Displays data from optical distance sensor.

 Bro. John       10/31/16 version 0.2 Just displays formatted raw and normalized
 reflected light levels.

 No gamepad input necessary. If the robot is being driven or manually moved, or if one
 moves objects in front of the sensor, those data will change.
 It assumes that the ODS sensor is configured with a name of "ods sensor".
 ****************************************************************************************/
@TeleOp(name = "Optical Distance Sensor", group = "Sensor")
//@Disabled
public class SensorMROpticalDistance extends LinearOpMode {
  OpticalDistanceSensor odsSensor;  // Hardware Device Object
  @Override
  public void runOpMode() throws InterruptedException {
    // get a reference to our Light Sensor object.
    odsSensor = hardwareMap.opticalDistanceSensor.get("ods");
    // wait for the start button to be pressed.
    waitForStart();
    // while the op mode is active, loop and read the light levels.
    // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
    while (opModeIsActive()) {
      // send the info back to driver station using telemetry function.
      telemetry.addData("Raw    ", "%7.4f", odsSensor.getRawLightDetected());
      telemetry.addData("Normal ", "%7.4f", odsSensor.getLightDetected());
      telemetry.update();
      // Call to idle() is not necessary with SDK version 2.35.
    }
  }
}