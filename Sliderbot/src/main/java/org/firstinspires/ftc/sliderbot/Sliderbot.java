package org.firstinspires.ftc.sliderbot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;

import java.util.ArrayList;
import java.util.List;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Sliderbot, which runs on a mecanum wheel drive train.
 * <p>
 * This hardware class assumes motor, sensor and servo names have been configured
 * in the robot configuration file.
 *
 * Version history
 * JMR v 0.1        Converted from Runnerbot to use mecanum wheel drive train, camera
 *   on port side. Works poorly, much wheel slippage.
 * JMR v 0.2        Camera mounted front. Still slipping badly, and sometimes gets
 *   so hopelessly lost it crashes into a wall.
 * JMR v 0.3 2/2/17 Camera back to starboard side.
 * JMR v 0.4 2/4/17 Restored stepToBeaconWestCoast, which was removed in conversion
 *   to mecanum drive. Our Sliderbot can use either one, but the West Coast version
 *   avoids the severe wheel slippage we get with strafing.
 * JMR v 0.5 2/8/17 Changed drive initialization methods to initEncodedDrive and
 *   initEncodedDrive. Tuned goToBeacon with variable approach speed.
 *
 */

public class Sliderbot {
    /* Public OpMode members. */
    // ** Some of these should have accessors.
    private static final double SQRT2 = Math.sqrt(2.0);
    private static final double COUNTS_PER_MOTOR_REV = 1120.0; // 1440 for TETRIX Encoder
    private static final int MAX_COUNTRATE = 2427; // max encoder rate for NeveRest 40. Was 2987.
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 3.96;   // calibrated
    public static final double DRIVEWHEEL_SEPARATION = 19.4;  // was 19.2
    public static double BOT_WIDTH = 17.8 * 25.4;  // Big Tile Runner chassis
    public static double BOT_LENGTH = 18.0 * 25.4;  // Big Tile Runner chassis
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    public static final double CAMERA_FROM_FRONT = 220.0; // Millimeters.
    public static final double TURN_SPEED = 0.5;
    public static final double STRAIGHT_SPEED = 0.6;
    public double sweepSpeed = 1.00;
    public boolean sweepToggle = false;
    public int sweepToggleState = 0;
    public DcMotor leftFrontMotor = null;
    public DcMotor leftRearMotor = null;
    public DcMotor rightFrontMotor = null;
    public DcMotor rightRearMotor = null;
    public DcMotor sweepMotor = null;
    public Servo buttonPush = null; // beacon button pusher
    public ColorSensor color = null;
    public I2cDevice range = null;
    private LinearOpMode currentOpMode;
    private LinearVisionOpMode currentVisionMode;
    public OpenGLMatrix lastLocation = null;
    public double knownX;
    public double knownY;
    public double knownHeading;
    public boolean seeking = false;
    public List<VuforiaTrackable> allTrackables = null;
    public VuforiaTrackables beacons = null;
    public boolean approaching = false;

    /* local robot members. */
    HardwareMap hwMap = null;

    /* Constructors */
    public Sliderbot() {};
    public Sliderbot(LinearOpMode linearOpMode) {
        currentOpMode = linearOpMode;
    }
    public Sliderbot(LinearVisionOpMode seeingOpMode) { currentVisionMode = seeingOpMode; }

    /***********************************************************************************
     * Robot initialization methods.
     ***********************************************************************************/

    /* Initialize all 4 motors to some RunMode. */
    public void initMotorsRunMode (DcMotor.RunMode someRunMode) {
        leftFrontMotor.setMode(someRunMode);
        leftRearMotor.setMode(someRunMode);
        rightFrontMotor.setMode(someRunMode);
        rightRearMotor.setMode(someRunMode);
    }

    /* Initialize all 4 motors to some behavior when they're told to stop. */
    public void initMotorsStopBehavior(DcMotor.ZeroPowerBehavior someBehavior) {
        leftFrontMotor.setZeroPowerBehavior(someBehavior);
        leftRearMotor.setZeroPowerBehavior(someBehavior);
        rightFrontMotor.setZeroPowerBehavior(someBehavior);
        rightRearMotor.setZeroPowerBehavior(someBehavior);
    }

    /* Initialize standard drive train equipment. */
    public void initUnencodedDrive(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFrontMotor = hwMap.dcMotor.get("leftFrontMotor");
        leftRearMotor = hwMap.dcMotor.get("leftRearMotor");
        rightFrontMotor = hwMap.dcMotor.get("rightFrontMotor");
        rightRearMotor = hwMap.dcMotor.get("rightRearMotor");

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);

        // Call initEncodedDrive if encoders are installed.
        // Generally, Autonomous modes will run encoded, and Driver (TeleOp) modes
        //   will run unencoded.
        initMotorsRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /*   Initialize drive equipment to use encoders.  CONVERTED. */
    public void initEncodedDrive(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFrontMotor = hwMap.dcMotor.get("leftFrontMotor");
        leftRearMotor = hwMap.dcMotor.get("leftRearMotor");
        rightFrontMotor = hwMap.dcMotor.get("rightFrontMotor");
        rightRearMotor = hwMap.dcMotor.get("rightRearMotor");

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontMotor.setMaxSpeed(MAX_COUNTRATE);
        leftRearMotor.setMaxSpeed(MAX_COUNTRATE);
        rightFrontMotor.setMaxSpeed(MAX_COUNTRATE);
        rightRearMotor.setMaxSpeed(MAX_COUNTRATE);
    }

    public void resetEncoderDrive() {
        // Zero the encoder counts targets. Side effect is to remove
        //   power, but we will do that explicitly.
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /*  Set up sweeper.  CONVERTED. */
    public void initSweeper () {
        sweepMotor = hwMap.dcMotor.get("sweeperMotor");
        sweepMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        sweepMotor.setPower(0);
    }

    public void initBeaconPusher () {
        buttonPush = hwMap.servo.get("buttonPush");
        buttonPush.setPosition(1.0);
    }

    //  ** To do: initGamePad() ... here or in opmode? Make a DriverStation class?

    //  ** To do: initSensors()

    /***********************************************************************************
     * Robot movement members.
     ***********************************************************************************/

    /*
     *  Most primitive movement members here. These send direct commands to the hardware.
     */

    /*  General movement. Most other movement methods will be wrappers for this.
     *    Specify speed and end condition for all four motors. Move will stop if
     *    either of two conditions occur:
     *  1) One of the four drive motors gets to the desired position.
     *  2) Driver quits the opmode. CONVERTED.
    */
    public void encoderDrive(double leftFrontSpeed, double rightFrontSpeed,
                             double leftRearSpeed, double rightRearSpeed,
                             double leftFrontInches, double rightFrontInches,
                             double leftRearInches, double rightRearInches) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;

        //  Discard current encoder positions.
        initMotorsStopBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        initMotorsRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Determine new target positions, and pass to motor controller
        newLeftFrontTarget =  (int) (leftFrontInches * COUNTS_PER_INCH);
        newRightFrontTarget = (int) (rightFrontInches * COUNTS_PER_INCH);
        leftFrontMotor.setTargetPosition(newLeftFrontTarget);
        rightFrontMotor.setTargetPosition(newRightFrontTarget);
        newLeftRearTarget =   (int) (leftRearInches * COUNTS_PER_INCH);
        newRightRearTarget =  (int) (rightRearInches * COUNTS_PER_INCH);
        leftRearMotor.setTargetPosition(newLeftRearTarget);
        rightRearMotor.setTargetPosition(newRightRearTarget);

        // Turn On RUN_TO_POSITION
        initMotorsRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Go!
        leftFrontMotor.setPower(Math.abs(leftFrontSpeed));
        rightFrontMotor.setPower(Math.abs(rightFrontSpeed));
        leftRearMotor.setPower(Math.abs(leftRearSpeed));
        rightRearMotor.setPower(Math.abs(rightRearSpeed));

        // keep looping while we are still active, and both motors are running.
        while (leftFrontMotor.isBusy() && rightFrontMotor.isBusy() &&
                leftRearMotor.isBusy() && rightRearMotor.isBusy() &&
                currentOpMode.opModeIsActive()) {
            //Just let the motors do their thing.
        }
    }

    /*
     *  Most primitive movement member. This is at the hardware primitives layer.
     *  CONVERTED.
    */
    public void stopDriveMotors(){
        leftFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
    }

    public void fullPowerDrive () {
        leftFrontMotor.setPower(1.0);
        leftRearMotor.setPower(1.0);
        rightFrontMotor.setPower(1.0);
        rightFrontMotor.setPower(1.0);
    }

    public void drivetrainPower(double power){
        leftFrontMotor.setPower(power);
        leftRearMotor.setPower(power);
        rightFrontMotor.setPower(power);
        rightRearMotor.setPower(power);
    }
    //  Command layer. Human driver issues commands with gamepad.
    //  CONVERTED, but not tempered.
    public String justDrive (){
        String report = "";
        double leftX = -currentOpMode.gamepad1.left_stick_x;
        double leftY = -currentOpMode.gamepad1.left_stick_y;
        double rightX = -currentOpMode.gamepad1.right_stick_x;
        double robotSpeed = Math.sqrt(leftX * leftX + leftY * leftY);
        double robotAngle = Math.atan2(leftY, -leftX) - Math.PI / 4;
        report = String.format(" Speed: %6.3f ", robotSpeed);
        report += String.format(" Course: %6.3f ", robotAngle);
        double leftFrontSpeed =  robotSpeed * Math.cos(robotAngle) - rightX;
        double rightFrontSpeed = robotSpeed * Math.sin(robotAngle) + rightX;
        double leftRearSpeed =   robotSpeed * Math.sin(robotAngle) - rightX;
        double rightRearSpeed =  robotSpeed * Math.cos(robotAngle) + rightX;

        leftFrontMotor.setPower(leftFrontSpeed);
        rightFrontMotor.setPower(rightFrontSpeed);
        leftRearMotor.setPower(leftRearSpeed);
        rightRearMotor.setPower(rightRearSpeed);

        return report;
    }
    /*
     *  All other movement members are at the command layer.
    */

    //  This one requires no command layer to hardware layer translation.
    //  Just continue going straight. CONVERTED.
    public void goStraight(double speed) {
        leftFrontMotor.setPower(speed);
        rightFrontMotor.setPower(speed);
        leftRearMotor.setPower(speed);
        rightRearMotor.setPower(speed);
    }

    //   Simple wrapper for encoderDrive. Just go straight a number of inches. CONVERTED.
    public void driveStraight(double speed, double inches){
        encoderDrive(speed, speed, speed, speed, inches, inches, inches, inches);
    }

    //  Just continue sliding sideways. Positive speed slides right.
    public void goSideways(double speed) {
        leftFrontMotor.setPower(-speed);
        rightFrontMotor.setPower(speed);
        leftRearMotor.setPower(-speed);
        rightRearMotor.setPower(speed);
    }

    //   Another simple wrapper. Just go sideways a number of inches. CONVERTED, but slips
    //     irregularly fore and aft.
    public void driveSideways (double speed, double inches) {
        encoderDrive (speed, speed, speed, speed, inches, -inches, -inches, inches);
    }

    //   CONVERTED, but wheel slippage can cause fore or aft drift.
    public void turnAngle (double speed, double angle) {
        double inches = angle * DRIVEWHEEL_SEPARATION/SQRT2;
        encoderDrive (speed, speed, speed, speed, -inches, inches, -inches, inches);
    }

    /*  Turning movements. All angles are in radians.  ** Convert to Meet 3 build. */
    //  Turn at speed through an angle, with a given radius.
    public void turnAngleRadiusDrive(double speed, double angle, double radius) {

        // One or both turning arcs could be negative.
      // ** implement as wrapper for encoderDrive
        // Degenerate cases: angle = 0, R = 0, R = d/2, R = +infinity
        // (straight drive).
    }

    //    Wrapper for turnAngleRadius
    // ** make this a wrapper for encoderDrive instead. ** Convert to Meet 3 build.
    public void turnArcRadiusDrive(double speed, double arc, double radius) {
        double targetAngle = arc / radius;
        turnAngleRadiusDrive(speed, targetAngle, radius);
    }

    //  Begin a left turn at speed, sharpness of turn decided by ratio.
    // ** Test on Interleague build.
    //    1:  go straight.
    //    0:  turn axis is left wheel.
    //    -1: turn axis is between drive wheels. Robot turns on own axis.
    public void steerLeft (double speed, double ratio) {
        Range.clip(ratio, -1.0, 1.0);
        leftFrontMotor.setPower(speed * ratio);
        leftRearMotor.setPower(speed * ratio);
        rightFrontMotor.setPower(speed);
        rightRearMotor.setPower(speed);
    }

    //  Right analog of steerLeft. ** Test on Meet 3 build.
    public void steerRight(double speed, double ratio) {
        Range.clip(ratio, -1.0, 1.0);
        leftFrontMotor.setPower(speed);
        leftRearMotor.setPower(speed);
        rightFrontMotor.setPower(speed * ratio);
        rightRearMotor.setPower(speed * ratio);
    }

    //  Drive a curved path by making left wheels turn slower and go
    //    shorter path by a factor of ratio. The right wheels will spin
    //    at parameter speed, and travel the full arc.
    public void turnLeft (double speed, double ratio, double arcInches) {
        Range.clip(ratio, -1.0, 1.0);
        encoderDrive(speed*ratio, speed, speed*ratio, speed,
                arcInches*ratio, arcInches, arcInches*ratio, arcInches);
    }

    //  Right analog of turnLeft.
    public void turnRight (double speed, double ratio, double arcInches) {
        Range.clip(ratio, -1.0, 1.0);
        encoderDrive(speed, speed*ratio, speed, speed*ratio,
                arcInches, arcInches*ratio, arcInches, arcInches*ratio);
    }

    /***********************************************************************************
     * Robot vision members.
     ***********************************************************************************/
    public VuforiaLocalizer.Parameters parameters;

    /* Start up Vuforia.  CONVERTED. */
    public VuforiaLocalizer initVuforia() {
        VuforiaLocalizer vuforia;
        parameters = new VuforiaLocalizer.Parameters(
                com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey =
                "ASkv3nr/////AAAAGYZ9CexhH0K0lDbV090F719DkwXCIXEUmExgnQNDFGjrDrkVJnU" +
                "7xNhuKHLsC32Pb1jmr+6vp6JtpVKvNmTf28ZYkUphDeajNPCLgGVxLjD6xsfgBayqSO" +
                "9bfQFeGkrdEgXlP+2oaz234afhWti9Jn8k71mzbQ4W2koX9yBMWz0YLzUWClcasxi6N" +
                "ty7SUvV+gaq3CzpKVtjKk+2EwV6ibIc0V47LAeB0lDGsGkSzuJ+93/Ulpoj+Lwr/jbI" +
                "2mu/Bs2W7U9mw73CMxvDix9o1FxyPNablla4W5C5lUDm0j2lW5gsUNOhgvlWKQ+eCu9" +
                "IBp53WbW5nfNzhXPaDDh/IlBbZuAMIJuMDEHI5PVLKT9L";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        //currentVisionMode.telemetry.addData("", "Vuforia initialized.");
        return vuforia;
    }

    //   CONVERTED.
    public List<VuforiaTrackable> initBeaconImages(VuforiaLocalizer someLocalizer) {
        /**
         * Load the data sets for the four beacon images we wish to track. These
         * particular data sets are stored in the 'assets' part of our application
         * (you'll see them in the Android Studio 'Project files' view over there on the
         * left of the screen, under FtcRobotController /src/main).
         * PDFs for the example "FTC_2016-17", datasets can be found in at
         * http://www.firstinspires.org/resource-library/ftc/game-and-season-info.
         */
        VuforiaTrackables beacons = someLocalizer.loadTrackablesFromAsset("FTC_2016-17");
        //currentVisionMode.telemetry.addData("", "Allocated beacons.");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Legos");
        beacons.get(3).setName("Gears");
        //currentVisionMode.telemetry.addData("", "Named beacons.");

        /* Assign locations to those images.
         * Just plaster all the images onto the middle of the Field, facing the Red Alliance
         *   Wall (X=0, Y=0). Later, we'll put them onto absolute positions as does the
         *   sample code ConceptVuforiaNavigation.
         *
         * To place an image Target at the origin, facing the Red Audience wall:
         *   We only need rotate it 90 around the field's X axis to flip it upright, and
         *   move it up to the level of the phone. We make one matrix to do that, and
         *   apply it to all the beacon images.
         */

        OpenGLMatrix imageTargetLocationOnField = OpenGLMatrix
                .translation(0, 0, (float)146.0) // Image height off floor
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, 0));
        beacons.get(0).setLocation(imageTargetLocationOnField);
        beacons.get(1).setLocation(imageTargetLocationOnField);
        beacons.get(2).setLocation(imageTargetLocationOnField);
        beacons.get(3).setLocation(imageTargetLocationOnField);
        //currentVisionMode.telemetry.addData("", "Located beacons.");

        /**
         * Create a transformation matrix describing where the phone is on the robot. Here, we
         * put the phone on the front middle of the robot with the screen facing in (see our
         * choice of BACK camera above) and in portrait mode. This requires no rotation of axes.
         */
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(0.0f, 0.0f, 0.0f); // Just locate the phone for now

        /**
         * Let the trackable listeners we care about know where the phone is. We know that each
         * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
         * we have not ourselves installed a listener of a different type.
         */
        ((VuforiaTrackableDefaultListener) beacons.get(0).getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener) beacons.get(1).getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener) beacons.get(2).getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener) beacons.get(3).getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

        /** Start tracking the beacon image data sets. Typically, we'll see none or one. */
        beacons.activate();
        /* For convenience, gather together all the trackable objects in one easily-iterable
         *   collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(beacons);
        return allTrackables;
    }

    //   CONVERTED.
    public String findBeaconImage(List<VuforiaTrackable> someImageSet) {
        String imageName = "nothing";
        for (VuforiaTrackable trackable : someImageSet) {
            /**
             * getUpdatedRobotLocation() will return null if no new information is available since
             * the last time that call was made, or if the trackable is not currently visible.
             * getRobotLocation() will return null if the trackable is not currently visible.
             */
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                imageName = trackable.getName();
                OpenGLMatrix robotLocationTransform =
                        ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                    break; // Never interested in more than one image.
                }
            }
        }
        return imageName;
    }

    //    Beacon approach, West Coast drive.
    private String stepTowardBeaconWestCoast (String whichBeacon) {
        String report = "";
        setXYHeading(lastLocation);
        double targetY = CAMERA_FROM_FRONT; // Millimeters. Stop when camera is in image.
        double remainingY = knownY - targetY; // Millimeters away from stopping.
        // Guidance tuning constants
        final double FAR_SPEED_FACTOR = 0.35;
        final double FINAL_APPROACH_SPEED = 0.45; // 0.1 is just a bit too slow.
        double approachSpeed = FINAL_APPROACH_SPEED + FAR_SPEED_FACTOR * remainingY/1000;
        // ** May become a tunable variable, slower for smaller abs(X).
        final double TURNER = 0.75; // Controls sharpness of turns.
        //  ** It may become a tunable variable, gentler for smaller abs(X).
        final double APPROACH_WHEEL_DIFFERENCE = TURNER * approachSpeed; // Steering
        final double HEADING_CORRECTOR = 3.0; // Controls strength of X correction

        // Angles in radians
        // knownHeading is zero if camera directly facing plane of the image, independent
        // of camera position.
        // Positive if pointing to left of normal to that plane through the camera.
        Orientation orientation = Orientation.getOrientation(lastLocation,
                AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        knownHeading = orientation.thirdAngle;

        // Zero if camera anywhere on normal to image, independent of camera orientation.
        // Positive if image is left of normal to plane of image, passing through the camera.
        double imageBearing = Math.atan2(-knownX, knownY);
        //double imageBearing = -knownX /knownY;
        imageBearing *= 180 / Math.PI;
        double targetHeading = imageBearing * HEADING_CORRECTOR;
        report += whichBeacon + String.format(" at X, Y: %4.0fmm", knownX) +
                String.format(", %4.0fmm", knownY);
        report += String.format("\nImage bearing %4.0f\u00b0", imageBearing);

        report += String.format("\nRobot heading: %4.0f\u00b0, speed %5.2f",
                knownHeading, approachSpeed);

        // Drive robot like an airplane landing on a runway. We have to be on its long
        //   axis, and pointed along that axis.
        // Steer toward this heading, attempting to line up
        if (knownHeading < targetHeading) {
            steerLeft (approachSpeed, 1.0 - TURNER);
            report += "Turning left to";
        } else {
            steerRight(approachSpeed, 1.0 - TURNER);
            report += "Turning right to";
        }

        report += String.format(" heading %4.0f\u00b0.", targetHeading);
        report += String.format(" %4.0fmm to go.", remainingY);
        if (remainingY < 0) {
            // Just stop, and let driver contemplate the situation.
            report += " I'm too close. Stopping and facing image.";
            //turnAngle(approachSpeed, - knownHeading);
            stopDriveMotors();
        }
        return report;
    }

    //   ** Converted, but sloppy due to wheel slip.
    private String stepTowardBeacon(String whichBeacon) {
        String report = "Approaching ";
        double leftFrontSpeed = 0.0;
        double rightFrontSpeed = 0.0;
        double leftRearSpeed = 0.0;
        double rightRearSpeed = 0.0;
        setXYHeading(lastLocation);
        double remainingX = knownX;
        double remainingY = knownY - CAMERA_FROM_FRONT;  // Millimeters away from stopping.

        // Angles in radians
        // knownHeading is zero if camera directly facing plane of the image, independent
        // of camera position.
        // Positive if pointing to left of normal to that plane through the camera.
        //Orientation orientation = Orientation.getOrientation(lastLocation,
        //        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        //knownHeading = orientation.thirdAngle;

        // Image bearing is zero if camera anywhere on normal to image, independent of camera
        //   orientation. Positive if image is left of normal to plane of image, passing through
        //   the camera.
        double imageBearing = -Math.atan2 (knownX, knownY);
        //imageBearing *= 180 / Math.PI;
        // Guidance tuning constants
        final double Y_CORRECTOR = 0.20; // ** .1 is too weak.
        //  ** May become a tunable variable, slower for smaller abs(X).
        final double X_CORRECTOR = 2.5; // Controls strength of X correction
        //  ** Also may become a tunable variable, gentler for smaller abs(X).
        final double HEADING_CORRECTOR = 0.35; // Controls strength of X correction
        double targetHeading = imageBearing * HEADING_CORRECTOR;
        report += whichBeacon + String.format(" at X, Y: %6.0fmm", knownX) +
                String.format(", %6.0fmm", knownY);
        report += String.format("\nImage bearing %6.2f radians", imageBearing);

        report += String.format("\nRobot heading %6.2f radians", knownHeading);

        /* Three components must be controlled.
             Y error: distance along normal to beacon image. Distance robot must still
               travel toward that image.
             X error: side to side error. Distance from that normal to the beacon image,
               approximately distance from center of white line.
             Heading error. Angle of robot's Y axis with respect to that normal to the
               beacon image.
           */
        //  Major component will be Y error. The others are perturbations from that.
        //    Just go forward. ** Looks good.

        leftFrontSpeed = Y_CORRECTOR;
        rightFrontSpeed = Y_CORRECTOR;
        leftRearSpeed = Y_CORRECTOR;
        rightRearSpeed = Y_CORRECTOR;

        //  X error correction. Modifies wheel commands that reduce Y error to produce
        //    side motion towards normal from beacon to wall, reducing sideways error.
        //    Strength of correction decays as X error decays. X error is measured by
        //    imageBearing.
        //
        leftFrontSpeed -= X_CORRECTOR * imageBearing * Y_CORRECTOR;
        leftRearSpeed += X_CORRECTOR * imageBearing * Y_CORRECTOR;
        rightFrontSpeed += X_CORRECTOR * imageBearing * Y_CORRECTOR;
        rightRearSpeed -= X_CORRECTOR * imageBearing * Y_CORRECTOR;

        //  Heading adjustment component. Modifies combined Y and X error corrections
        //    to turn robot. This correction also decays, governed by knownHeading.
        //
        leftFrontSpeed += HEADING_CORRECTOR * knownHeading * Y_CORRECTOR;
        leftRearSpeed += HEADING_CORRECTOR * knownHeading * Y_CORRECTOR;
        rightFrontSpeed -= HEADING_CORRECTOR * knownHeading * Y_CORRECTOR;
        rightRearSpeed -= HEADING_CORRECTOR * knownHeading * Y_CORRECTOR;

        //  Apply all 3 corrections to robot wheels.
        leftFrontMotor.setPower(leftFrontSpeed);
        leftRearMotor.setPower(leftRearSpeed);
        rightFrontMotor.setPower(rightFrontSpeed);
        rightRearMotor.setPower(rightRearSpeed);

        report += String.format("\nWheel speeds: lf %6.2f  rf %6.2f", leftFrontSpeed, rightFrontSpeed);
        report += String.format(" lr %6.2f  rr %6.2f", leftRearSpeed,  rightRearSpeed);
        if (remainingY < 0) {
            // Just report, and let driver contemplate the situation.
            double approachSpeed = (leftFrontSpeed + leftRearSpeed +
                    rightFrontSpeed + rightRearSpeed)/4.0;
            report += " I'm too close. Stopping and facing image.";
            turnAngle(approachSpeed, - knownHeading);
            stopDriveMotors();
        }
        RobotLog.a (report);
        return report;
    }

    //   ** Test on Meet 3 build with converted stepTowardBeacon.
    public void goToBeacon (){
        do {
            String field = "I see";
            String imageName = "nothing";
            String report = "";

            //  Find an image, and get robot's location relative to it.
            //  ** in Autonomous opmode, fails to find 2nd beacon image after successfully
            //    finding the first. It still thinks it sees that first one.
            imageName = findBeaconImage(allTrackables);

            if (! imageName.equals("nothing")) {
                //  Go toward detected beacon image.
                report = stepTowardBeaconWestCoast(imageName);
                //report = stepTowardBeacon(imageName);
                if (knownY < CAMERA_FROM_FRONT) {
                    report += "Arrived.";
                    stopDriveMotors();
                    currentOpMode.telemetry.addData(field, report);
                    currentOpMode.telemetry.update();
                    break;
                }
            } else {
                report = "nothing. Stopping.";
                // Stop
                // ** Maybe turn robot toward last image bearing, hoping to pick it up again
                stopDriveMotors();
            }

            currentOpMode.telemetry.addData(field, report);
            currentOpMode.telemetry.update();
        } while (true);
    }

/***********************************************************************************
 *          Robot navigation members.
 ***********************************************************************************/

    //  Set robot heading and coordinates from some location object.
    //  ** Convert to Meet 3 build.
    public void setXYHeading(OpenGLMatrix aLocation){
        VectorF translation = aLocation.getTranslation();
        knownX = -translation.get(0);
        knownY = -translation.get(1);
        Orientation orientation = Orientation.getOrientation(lastLocation,
                AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
        knownHeading = orientation.thirdAngle;
    }
}