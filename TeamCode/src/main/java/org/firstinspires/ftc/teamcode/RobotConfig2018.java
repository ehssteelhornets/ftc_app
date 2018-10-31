package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotConfig2018
        {
        /* Public OpMode members. */
        public DcMotor leftDrive   = null;
        public DcMotor  rightDrive  = null;
        public DcMotor  leftDrive2   = null;
        public DcMotor  rightDrive2  = null;
        public DcMotor screw = null;
        public DcMotor climb = null;
        public DcMotor reel = null;
        public Servo mServo = null;
        public Servo sServo = null;

        public ColorSensor sensor_color = null;
        /* local OpMode members. */
        HardwareMap hwMap           =  null;
        private ElapsedTime period  = new ElapsedTime();
        /* Constructor */
        public RobotConfig2018(){

        }
        /* Initialize standard Hardware interfaces */
        public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        leftDrive2 = hwMap.get(DcMotor.class, "left_drive2");
        rightDrive2 = hwMap.get(DcMotor.class, "right_drive2");
        screw = hwMap.get(DcMotor.class,"screw");
        climb = hwMap.get(DcMotor.class,"climb");
        reel = hwMap.get(DcMotor.class,"reel");

         mServo= hwMap.get(Servo.class,"mServo");
         sServo = hwMap.get(Servo.class,"sServo");
        //Set motor direction
        leftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftDrive2.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightDrive2.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        climb.setDirection(DcMotor.Direction.FORWARD);
        screw.setDirection(DcMotor.Direction.FORWARD);
        reel.setDirection(DcMotor.Direction.FORWARD);
        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        rightDrive2.setPower(0);
        leftDrive2.setPower(0);
        climb.setPower(0);
        screw.setPower(0);
        reel.setPower(0);
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        screw.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        climb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        reel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Define and initialize ALL installed servos
            //mServo.setPosition(0);
            //sServo.setPosition(0);
        //Define the color sensor
        //sensor_color = hwMap.get(ColorSensor.class, "sensor_color_distance");
        }
        }
