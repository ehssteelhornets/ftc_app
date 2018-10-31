package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
//import com.disnodeteam.dogecv.DogeCV;
//import com.disnodeteam.dogecv.CameraViewDisplay;
//import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
@Autonomous(name="AutoBlue118", group ="Pushbot")
public class AutoBlue118 extends LinearOpMode {
    RobotConfig2018 robot = new RobotConfig2018();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    // eg: AndyMark Orbital 20 Motor Encoder
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: AndyMark Orbital 20 Motor Encoder from Video
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP AndyMark Orbital 20
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    //Color Sensor initialization
    ColorSensor sensorColor;
    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;
    @Override
    public void runOpMode() {
    //Initializes the robot
        robot.init(hardwareMap);
        //DogeCV init
  //      GoldAlignDetector goldAlignDetector = new GoldAlignDetector();
    //    goldAlignDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
      //  goldAlignDetector.useDefaults();
        //goldAlignDetector.enable();
        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        //Reset our encoders
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
        waitForStart();
        //Unlatch
                        //Remove latch
                        //Rack and pinion up
                        //Strafe left
                        //Drive 2

        //telemetry.addData("IsAligned" , goldAlignDetector.getAligned()); // Is the bot aligned with the gold mineral
        //telemetry.addData("X Pos" , goldAlignDetector.getXPosition()); // Gold X pos.


        /**if(!goldAlignDetector.getAligned())
        {
            telemetry.addLine("NOOOOO");
            telemetry.update();
        }
        else
            {
            //Aligned!
            telemetry.addLine("YASSSS");
            telemetry.update();
        }
         **/
            //Drive to depot
        encoderDrive(1,20,20,5.0 );
        //Drop Marker
        //robot.tServo.setPosition(90);
        //Drive to sampling field
        encoderDrive(1, -35 , -35, 5.0);
        //Turn n degrees to run parralel to samples
        encoderDrive(1, 10, 0,  5.0);
        //go to first sample
        encoderDrive(1, 10, 10, 5.0);
        //sample stuff
        boolean isFound = false;
       while(!isFound) {

           Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                   (int) (sensorColor.green() * SCALE_FACTOR),
                   (int) (sensorColor.blue() * SCALE_FACTOR),
                   hsvValues);
           if (hsvValues[0] > 35) {
               telemetry.addData("Object:", "Ball");
               telemetry.update();
           } else {
               telemetry.addData("Object:", "Cube");
               telemetry.update();
               //robot.cServo.setPosition(90);
               encoderDrive(1, 5,5,5.0);
               isFound = true;
           }
       }
       //drive back to the wall
        encoderDrive(1,-25,-25,5.0);
        //turn -n degree
        encoderDrive(1,-10, 0, 5.0);
        //Park in crater
        encoderDrive(1,10,10,5.0);
        //Done

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

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    public void hangDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.climb.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            robot.climb.setTargetPosition(newLeftTarget);
            // Turn On RUN_TO_POSITION
            robot.climb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            robot.climb.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) && robot.reel.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.climb.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.climb.setPower(0);

            robot.climb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}

