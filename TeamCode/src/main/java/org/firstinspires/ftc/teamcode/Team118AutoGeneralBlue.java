package org.firstinspires.ftc.teamcode;

        import android.graphics.Color;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DigitalChannel;
        import com.qualcomm.robotcore.hardware.DistanceSensor;
        import com.qualcomm.robotcore.util.ElapsedTime;

        import org.firstinspires.ftc.robotcore.external.ClassFactory;
        import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
        import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
        import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
        import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
        import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

//import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;

@Autonomous(name="Team118AutoRed", group ="Pushbot")
public class Team118AutoGeneralBlue extends LinearOpMode {
    /* Declare OpMode members. */
    NewHardwarePushbot robot = new NewHardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    // eg: AndyMark Orbital 20 Motor Encoder
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: AndyMark Orbital 20 Motor Encoder from Video

    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP AndyMark Orbital 20

    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference

    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    int vu = 0;

    DistanceSensor sensorDistance;
    DigitalChannel digitalTouch;  // Hardware Device Object
    //Sensor initilizations
    float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;

    OpenGLMatrix lastLocation = null; // WARNING: VERY INACCURATE, USE ONLY TO ADJUST TO FIND IMAGE AGAIN! DO NOT BASE MAJOR MOVEMENTS OFF OF THIS!!
    double tX; // X value extracted from our the offset of the traget relative to the robot.
    double tZ; // Same as above but for Z
    double tY; // Same as above but for Y
    // -----------------------------------
    double rX; // X value extracted from the rotational components of the tartget relitive to the robot
    double rY; // Same as above but for Y
    double rZ; // Same as above but for Z
    private int Rtimes = 0;
    private int Btimes = 0;
    private int times = 0;
    VuforiaLocalizer vuforia;
//*********************************************************************************************************

    @Override
    public void runOpMode() {
//Initiliazes the robot
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition());
        telemetry.update();

        //Vuforia
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AVBEWl3/////AAAAGbcR3zLzokcgvluieL+X7sUZchFxSNixRNcAWxR0bP9U+U43LsdX44KA/uRn41WeKGbwe4gTQcOdJve3abc+3SFCDWjM5NdDbjZkEutO2JcIigwn7jIU41jL6sXmCekHzJ7tW8F2B1JfISG6WP9KpbcD9F9BfMHnvBljUyT8nLP89/pqN0r8Zy2L5n9avC/LchzRCsMnvalZKZyYJkmlfNS8o4lKSOGzP2iEWx5a5J02jJiAwgPmsIjGKBWpUdwqB4fRlLLNxUMXKtRBIMEaLPn1+tdjMIIQX/fdf7q50MIWPdTdDdKJlbVHCiGrQa46ad5SA2+hFfsCglv8GW30Peuom9O5lGOWzjxSdcAv/H2W";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK; // Use FRONT Camera (Change to BACK if you want to use that one)
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES; // Display Axes

        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTrackables.activate(); // Activate Vuforia
        //Waits till play is hit
        waitForStart();

        //Vuforia tests
        sleep(1000);
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) { // Test to see if image is visable
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose(); // Get Positional value to use later
            telemetry.addData("Pose", format(pose));
            if (pose != null) {
                VectorF trans = pose.getTranslation();
                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Extract the X, Y, and Z components of the offset of the target relative to the robot
                tX = trans.get(0);
                tY = trans.get(1);
                tZ = trans.get(2);

                // Extract the rotational components of the target relative to the robot. NOTE: VERY IMPORTANT IF BASING MOVEMENT OFF OF THE IMAGE!!!!
                rX = rot.firstAngle;
                rY = rot.secondAngle;
                rZ = rot.thirdAngle;
            }
            if (vuMark == RelicRecoveryVuMark.LEFT) { // Test to see if Image is the "LEFT" image and display value.
                telemetry.addData("VuMark is", "Left");
                telemetry.addData("X =", tX);
                telemetry.addData("Y =", tY);
                telemetry.addData("Z =", tZ);
                vu = 3;
            } else if (vuMark == RelicRecoveryVuMark.RIGHT) { // Test to see if Image is the "RIGHT" image and display values.
                telemetry.addData("VuMark is", "Right");
                telemetry.addData("X =", tX);
                telemetry.addData("Y =", tY);
                telemetry.addData("Z =", tZ);
                vu = 1;
            } else if (vuMark == RelicRecoveryVuMark.CENTER) { // Test to see if Image is the "CENTER" image and display values.
                telemetry.addData("VuMark is", "Center");
                telemetry.addData("X =", tX);
                telemetry.addData("Y =", tY);
                telemetry.addData("Z =", tZ);
                vu = 2;

            }
        } else {
            telemetry.addData("VuMark", "not visible");
            vu = 2;
        }
        telemetry.update();

        //End of vuforia
        sleep(1000);
        // Step through each portion of the path,
        robot.cServo.setPosition(0.0);
        //color decision statement for red
        sleep(500);
        telemetry.addData("RED Color value: ", robot.sensor_color.red());
        telemetry.addData("Blue Color value: ", robot.sensor_color.blue());

        while (Rtimes < 5 && Btimes < 5 && times < 10) {
            robot.sensor_color.enableLed(true);
            telemetry.addData("RED Color value: ", robot.sensor_color.red());
            telemetry.addData("Blue Color value: ", robot.sensor_color.blue());
            telemetry.addData("RED : ", Rtimes);
            telemetry.addData("Blue : ", Btimes);
            telemetry.addData("Green Color Value : ", robot.sensor_color.green());
            Color.RGBToHSV((int) (robot.sensor_color.red() * SCALE_FACTOR),
                    (int) (robot.sensor_color.green() * SCALE_FACTOR),
                    (int) (robot.sensor_color.blue() * SCALE_FACTOR),
                    hsvValues);
            //We are Red team- HUE CODE WORKS!!!! DO NOT TOUCH UNDER ANY CIRCUMSTANCE
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.update();
            if(hsvValues[0] >= 55 && hsvValues[0] <= 255 ) {
                Btimes++;
                telemetry.addData("Color: ", "Blue");
                telemetry.update();
            } else if (hsvValues[0] <= 30 || hsvValues[0] >= 331)
            {
                Rtimes++;
                telemetry.addData("Color: ", "Red");
                telemetry.update();
            }
            sleep(500);
            times++;
        }
        if (Rtimes >= 5) {
// red ball is detected so hit blue ball by going forwards
            encoderDrive(TURN_SPEED, -5, 5, 5.0);
            robot.cServo.setPosition(1.0);
            encoderDrive(TURN_SPEED, 5, -5, 5.0);
        } else if (Btimes >= 5) {
// blue ball is detected so hit blue ball by going backwards
            encoderDrive(TURN_SPEED, 5, -5, 5.0);
            robot.cServo.setPosition(1.0);
            encoderDrive(TURN_SPEED, -5, 5, 5.0);
        } else if (times >= 10) {
            robot.cServo.setPosition(1.0);
        }
        telemetry.update();
        robot.cServo.setPosition(1.0);
        sleep(500);
        //Drive to the box
        if (vu == 1) //Drive Right
        {
            encoderDrive(DRIVE_SPEED, -22, -22, 5.0);
        } else if (vu == 2) //Drive Center
        {
            encoderDrive(DRIVE_SPEED, -24.75, -24.75, 5.0);
        } else if (vu == 3) //Drive Left
        {
            encoderDrive(DRIVE_SPEED, -29, -29, 5.0);
        }
        sleep(1000);
        //Turn 90 degrees
        encoderDrive(DRIVE_SPEED,-8,8, 5.0);
        //Drive to box
        encoderDrive(DRIVE_SPEED,-5,-5,5.0);
        robot.rFlip.setPosition(0.7);
        robot.lFlip.setPosition(0.7);
        //Back up 10 inches
        sleep(500);
        robot.lFlip.setPosition(0.0);
        robot.rFlip.setPosition(0.0);
        sleep(500);
        robot.rIntake.setPower(1.0);
        robot.lIntake.setPower(1.0);
        encoderDrive(DRIVE_SPEED,27,27,5.0);
        sleep(1000);
        encoderDrive( DRIVE_SPEED,-27,-27,5.0);
        eleDrive(TURN_SPEED,0.5,1.0);
        robot.rIntake.setPower(0.0);
        robot.lIntake.setPower(0.0);
        robot.rFlip.setPosition(0.7);
        robot.lFlip.setPosition(0.7);
        sleep(500);
        robot.rFlip.setPosition(0.0);
        robot.lFlip.setPosition(0.0);
        encoderDrive(DRIVE_SPEED,2,2,5.0);
        sleep(500);
    }


//Encoder method
// Note: Reverse movement is obtained by setting a negative distance (not speed)

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newLeftTarget2;
        int newRightTarget2;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newLeftTarget2 = robot.leftDrive2.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget2 = robot.rightDrive2.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);
            //robot.leftDrive2.setTargetPosition(newLeftTarget2);
            //robot.rightDrive2.setTargetPosition(newRightTarget2);

            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower(Math.abs(speed));
            //robot.leftDrive2.setPower(Math.abs(speed));
            //robot.rightDrive2.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);
            robot.leftDrive2.setPower(0);
            robot.rightDrive2.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    public void eleDrive(double speed,
                         double leftInches, double timeoutS) {
        int newLeftTarget3;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget3 = robot.elevator.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);

            robot.elevator.setTargetPosition(newLeftTarget3);

            // Turn On RUN_TO_POSITION
            robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            robot.elevator.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.elevator.isBusy())) {

                // Display it for the driver.
                //telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget3);
                //telemetry.addData("Path2", "Running at %7d :%7d",
                //        robot.elevator.getCurrentPosition());
                telemetry.addData("Elevator running", "true");
                telemetry.update();
            }
            // Stop all motion;
            robot.elevator.setPower(0);
            // Turn off RUN_TO_POSITION
            robot.elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
    public void chroma()
    {
        if (robot.sensor_color.red() > 25 && robot.sensor_color.red() > robot.sensor_color.blue())
        //if(hsvValues[0] >= 150 && hsvValues[0] <= 275)
        {
            Rtimes++;
        } else if (robot.sensor_color.blue() > 25 && robot.sensor_color.red() < robot.sensor_color.blue()) {
            Btimes++;
        }
    }
}

