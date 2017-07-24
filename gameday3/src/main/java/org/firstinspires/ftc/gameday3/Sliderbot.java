package org.firstinspires.ftc.gameday3;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
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
//mport org.firstinspires.ftc.sliderbot.VVField;
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
 */

public class Sliderbot {
    /* Public OpMode members. */
    private static final double SQRT2 = Math.sqrt(2.0);
    private static final double COUNTS_PER_MOTOR_REV = 1120.0; // use 1440 for TETRIX Motor Encoder
    private static final double DRIVE_GEAR_REDUCTION = 1.0;    // 2.0 for Pushbot, < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 3.96;   // calibrated
    //  ** Some of these should be accessors.
    public static final double DRIVEWHEEL_SEPARATION = 19.2;  // calibrated inches
    public static double BOT_WIDTH = 17.8 * 25.4;  // Big Tile Runner chassis
    public static double BOT_LENGTH = 18.0 * 25.4;  // Big Tile Runner chassis
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    public static final double CAMERA_FROM_FRONT = 244.0; // Millimeters. ** Convert to Meet 3 build
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
    //private VVField playingOnField;
    public OpenGLMatrix lastLocation = null;
    public double knownX;
    public double knownY;
    public double knownHeading;
    public boolean seeking = false;
    public List<VuforiaTrackable> allTrackables = null;

    /* local robot members. */
    HardwareMap hwMap = null;

    /* Constructor */
    public Sliderbot(LinearOpMode linearOpMode) {
        currentOpMode = linearOpMode;
    }
    public Sliderbot(LinearVisionOpMode seeingOpMode) { currentVisionMode = seeingOpMode; }
    public Sliderbot() {};

    /***********************************************************************************
     * Robot initialization methods.
     ***********************************************************************************/
    /* Initialize standard drive train equipment. CONVERTED. */
    public void initDrive (HardwareMap ahwMap) {
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

        // May want to use RUN_USING_ENCODERS if encoders are installed.
        // Generally, Autonomous modes will run encoded, and Driver (TeleOp) modes
        //   will run unencoded.
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /*   Initialize drive equipment to use encoders.  CONVERTED. */
    public void initEncodedDrive() {
        // Zero the encoder counts targets. Side effect is to remove
        //   power, but we will do that explicitly.
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /*  Set up servos and sensors  CONVERTED. */
    public void initSweeper () {
        sweepMotor = hwMap.dcMotor.get("sweeperMotor");
        sweepMotor.setPower(0);
    }

    public void initBeaconPusher () { buttonPush = hwMap.servo.get("buttonPush");
    }

    /***********************************************************************************
     * Robot movement members.
     ***********************************************************************************/

    /*
     *  Most primitive movement members here. These send direct commands to the hardware.
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

        // Determine new target positions, and pass to motor controller
        newLeftFrontTarget = leftFrontMotor.getCurrentPosition() +
                (int) (leftFrontInches * COUNTS_PER_INCH);
        newRightFrontTarget = rightFrontMotor.getCurrentPosition() +
                (int) (rightFrontInches * COUNTS_PER_INCH);
        leftFrontMotor.setTargetPosition(newLeftFrontTarget);
        rightFrontMotor.setTargetPosition(newRightFrontTarget);
        newLeftRearTarget = leftRearMotor.getCurrentPosition() +
                (int) (leftRearInches * COUNTS_PER_INCH);
        newRightRearTarget = rightRearMotor.getCurrentPosition() +
                (int) (rightRearInches * COUNTS_PER_INCH);
        leftRearMotor.setTargetPosition(newLeftRearTarget);
        rightRearMotor.setTargetPosition(newRightRearTarget);

        // Turn On RUN_TO_POSITION
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Go!
        leftFrontMotor.setPower(Math.abs(leftFrontSpeed));
        rightFrontMotor.setPower(Math.abs(rightFrontSpeed));
        leftRearMotor.setPower(Math.abs(leftRearSpeed));
        rightRearMotor.setPower(Math.abs(rightRearSpeed));

        // keep looping while we are still active, and both motors are running.
        while (leftFrontMotor.isBusy() && rightFrontMotor.isBusy() &&
                leftRearMotor.isBusy() && rightRearMotor.isBusy() &&
                currentOpMode.opModeIsActive()) {
            // just let it run
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
        currentOpMode.telemetry.addData("Course", robotAngle);
        currentOpMode.telemetry.update();
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

    //  Begin a left turn at speed, sharpness of turn decided by ratio. ** Test on Meet 3 build.
    //    1:  go straight.
    //    0:  turn axis is left wheel.
    //    -1: turn axis is between drive wheels. Robot turns on own axis.
    public void turnLeft(double speed, double ratio) {
        Range.clip(ratio, -1.0, 1.0);
        leftFrontMotor.setPower(speed * ratio);
        leftRearMotor.setPower(speed * ratio);
        rightFrontMotor.setPower(speed);
        rightRearMotor.setPower(speed);
    }

    //  Right analog of turnLeft. ** Test on Meet 3 build.
    public void turnRight(double speed, double ratio) {
        Range.clip(ratio, -1.0, 1.0);
        leftFrontMotor.setPower(speed);
        leftRearMotor.setPower(speed);
        rightFrontMotor.setPower(speed * ratio);
        rightRearMotor.setPower(speed * ratio);
    }

    //  ** Test on Meet 3 build.
    public void turnOnSelf (double speed, double angle) {
        encoderDrive(speed, speed, speed, speed,
                DRIVEWHEEL_SEPARATION*angle, -DRIVEWHEEL_SEPARATION*angle,
                DRIVEWHEEL_SEPARATION*angle, -DRIVEWHEEL_SEPARATION*angle);
    }

    /***********************************************************************************
     * Robot vision members.
     ***********************************************************************************/
    public VuforiaLocalizer.Parameters parameters;

    /* Start up Vuforia.  CONVERTED. */
    public VuforiaLocalizer initVuforia() {
        VuforiaLocalizer vuforia;
        parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "ASkv3nr/////AAAAGYZ9CexhH0K0lDbV090F719DkwXCIXEUmExgnQNDFGjrDrkVJnU7xNhuKHLsC32Pb1jmr+6vp6JtpVKvNmTf28ZYkUphDeajNPCLgGVxLjD6xsfgBayqSO9bfQFeGkrdEgXlP+2oaz234afhWti9Jn8k71mzbQ4W2koX9yBMWz0YLzUWClcasxi6Nty7SUvV+gaq3CzpKVtjKk+2EwV6ibIc0V47LAeB0lDGsGkSzuJ+93/Ulpoj+Lwr/jbI2mu/Bs2W7U9mw73CMxvDix9o1FxyPNablla4W5C5lUDm0j2lW5gsUNOhgvlWKQ+eCu9IBp53WbW5nfNzhXPaDDh/IlBbZuAMIJuMDEHI5PVLKT9L";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        return vuforia;
    }

    //   CONVERTED.
    public List<VuforiaTrackable> initBeaconImages(VuforiaLocalizer someLocalizer) {
        /**
         * Load the data sets for the four beacon images we wish to track. These particular data
         * sets are stored in the 'assets' part of our application (you'll see them in the Android
         * Studio 'Project files' view over there on the left of the screen, under FtcRobotController
         * /src/main).
         * PDFs for the example "FTC_2016-17", datasets can be found in at
         * http://www.firstinspires.org/resource-library/ftc/game-and-season-info.
         */
        VuforiaTrackables beacons = someLocalizer.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Legos");
        beacons.get(3).setName("Gears");

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

    //   ** CONVERTED, but sloppy due to wheel slip.
    private String stepTowardBeacon(String whichBeacon) {
        String report = "";
        //double leftFrontSpeed = 0.0;
        //double rightFrontSpeed = 0.0;
        //double leftRearSpeed = 0.0;
        //double rightRearSpeed = 0.0;
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
        final double APPROACH_SPEED = 0.20; // ** .1 is too weak.
        //  ** May become a tunable variable, slower for smaller abs(X).
        final double FORE_AFTER = 2.5; // Controls strength of X correction
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
        //    Slide to port. ** Looks good.

        double leftFrontSpeed = -APPROACH_SPEED;
        double rightFrontSpeed = APPROACH_SPEED;
        double leftRearSpeed = APPROACH_SPEED;
        double rightRearSpeed = -APPROACH_SPEED;

        //  X error correction. Modifies wheel commands that reduce Y error to produce
        //    angled motion towards beacon, reducing sideways error. Strength of correction
        //    decays as X error decays. X error is measured by imageBearing.
        // ** Looks good
        leftFrontSpeed -= FORE_AFTER * imageBearing * APPROACH_SPEED;
        leftRearSpeed -= FORE_AFTER * imageBearing * APPROACH_SPEED;
        rightFrontSpeed -= FORE_AFTER * imageBearing * APPROACH_SPEED;
        rightRearSpeed -= FORE_AFTER * imageBearing * APPROACH_SPEED;

        //  Heading adjustment component. Modifies combined Y and X error corrections
        //    to turn robot. This correction also decays, governed by knownHeading.
        //    ** Looks good.
        leftFrontSpeed += HEADING_CORRECTOR * knownHeading * APPROACH_SPEED;
        leftRearSpeed += HEADING_CORRECTOR * knownHeading * APPROACH_SPEED;
        rightFrontSpeed -= HEADING_CORRECTOR * knownHeading * APPROACH_SPEED;
        rightRearSpeed -= HEADING_CORRECTOR * knownHeading * APPROACH_SPEED;

        //  Apply all 3 corrections to robot wheels.
        leftFrontMotor.setPower(leftFrontSpeed);
        leftRearMotor.setPower(leftRearSpeed);
        rightFrontMotor.setPower(rightFrontSpeed);
        rightRearMotor.setPower(rightRearSpeed);

        report += String.format("\nWheel speeds: lf %6.2f  rf %6.2f", leftFrontSpeed, rightFrontSpeed);
        report += String.format(" lr %6.2f  rr %6.2f", leftRearSpeed,  rightRearSpeed);
        if (remainingY < 0) {
            // Just report, and let driver contemplate the situation.
            report += " I'm too close. Stopping.";
            // stopDriveMotors();
        }
        RobotLog.a (report);
        return report;
    }

    //   ** CONVERTED, works
    public void goToBeacon (){
        do {
            String field = "I see";
            String imageName = "nothing";
            String report = "";

            //  Find an image, and get robot's location relative to it.
            imageName = findBeaconImage(allTrackables);

            if (! imageName.equals("nothing")) {
                //  Go toward detected beacon image.
                report = stepTowardBeacon(imageName);
                if (knownY < CAMERA_FROM_FRONT) {
                    report += "Arrived. Stopping.";
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
    //  ** CONVERTED.
    public void setXYHeading(OpenGLMatrix aLocation){
        VectorF translation = aLocation.getTranslation();
        knownX = -translation.get(0);
        knownY = -translation.get(1);
        Orientation orientation = Orientation.getOrientation(lastLocation,
                AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
        knownHeading = orientation.thirdAngle;
    }
}