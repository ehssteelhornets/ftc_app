/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class NewHardwarePushbot
{
    /* Public OpMode members. */
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor  leftDrive2   = null;
    public DcMotor  rightDrive2  = null;
    public DcMotor  rIntake         = null;
    public DcMotor  lIntake         = null;
    public DcMotor reel = null;
    public DcMotor elevator = null;
    public Servo    leftClaw    = null;
    public Servo    rightClaw   = null;
    public Servo    lFlip    = null;
    public Servo    rFlip   = null;
    public Servo cServo = null;
    public Servo cam = null;
    public ColorSensor sensor_color = null;

    public static final double MID_SERVO       =  0.5 ;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public NewHardwarePushbot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        leftDrive2 = hwMap.get(DcMotor.class, "right_drive2");
        rightDrive2 = hwMap.get(DcMotor.class, "left_drive2");
        rIntake = hwMap.get(DcMotor.class, "rIntake");
        lIntake = hwMap.get(DcMotor.class,"lIntake");
        reel = hwMap.get(DcMotor.class,"reel");
        elevator = hwMap.get(DcMotor.class,"elevator");
        leftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftDrive2.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightDrive2.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        lIntake.setDirection(DcMotor.Direction.FORWARD);
        rIntake.setDirection(DcMotor.Direction.REVERSE);
        reel.setDirection(DcMotor.Direction.FORWARD);
        elevator.setDirection(DcMotor.Direction.FORWARD);
        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        rightDrive2.setPower(0);
        leftDrive2.setPower(0);
        rIntake.setPower(0);
        lIntake.setPower(0);
        reel.setPower(0);
        elevator.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        reel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Define and initialize ALL installed servos.
        //leftClaw  = hwMap.get(Servo.class, "left_hand");
        //rightClaw = hwMap.get(Servo.class, "right_hand");
        lFlip  = hwMap.get(Servo.class, "lFlipper");
        rFlip = hwMap.get(Servo.class, "rFlipper");
        cServo = hwMap.get(Servo.class, "color_servo");
        //rightClaw = hwMap.get(Servo.class, "right_claw");
        //cam = hwMap.get(Servo.class, "Cam");
        //rightClaw.setPosition(0.0);//GOOD (O is .75) (C is .65)
        //O means open, C means closed
        //We want to start closed so we can hold a glyph at the start of auto.
        cServo.setPosition(1.0);
        //cam.setPosition(0.5);
        sensor_color = hwMap.get(ColorSensor.class, "sensor_color_distance");
        rFlip.setPosition(0.0);
        lFlip.setPosition(0.0);
    }
}
