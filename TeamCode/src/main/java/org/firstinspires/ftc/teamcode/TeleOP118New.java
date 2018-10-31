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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robocol.TelemetryMessage;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOP 118New", group="Linear Opmode")

public class TeleOP118New extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftDrive2 = null;
    private DcMotor rightDrive2 = null;
    private DcMotor arm = null;
    public Servo leftClaw    = null;
    public Servo    rightClaw   = null;
    public static final double MID_SERVO       =  0.5 ;
    static double right;
    static double left;
    static double right2;
    static double left2;
    NewHardwarePushbot robot = new NewHardwarePushbot();

    double          clawOffset      = 0;
    @Override
    public void runOpMode() {

// Save reference to Hardware map


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.init(hardwareMap);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        //telemetry.addData("Picture: ", R.drawable.dolphin);

        // run until the end of the match (driver presses STOP)

        while (opModeIsActive()) {
            //robot.cServo.setPosition(0.6);
            // Setup a variable for each drive wheel to save power level for telemetry
            right = gamepad1.right_stick_y;
            left = gamepad1.left_stick_y;
            boolean precisionMode = false;
            if (gamepad1.left_trigger != 0)
                precisionMode = true;
            boolean reverseMode = false;
            if (gamepad1.right_trigger != 0)
                reverseMode = true;
            drive(precisionMode, reverseMode);
            //Intake Controls
            if (gamepad1.a) {
                robot.rIntake.setPower(1.0);
                robot.lIntake.setPower(1.0);
            }
            if (gamepad1.b) {
                robot.rIntake.setPower(0.0);
                robot.lIntake.setPower(0.0);
            }
            //Spit function
            if (gamepad1.right_bumper) {
                robot.rIntake.setPower(-1.0);
                robot.lIntake.setPower(-1.0);
            }
            //Flipping Plate Controls
            if (gamepad1.x) {
                robot.rFlip.setPosition(0.5);
                robot.lFlip.setPosition(0.5);
            }
            if (gamepad1.y) {
                robot.rFlip.setPosition(0.0);
                robot.lFlip.setPosition(0.0);
            }
            if(gamepad1.dpad_down)
            {
                robot.rFlip.setPosition(robot.rFlip.getPosition() - 0.01);
                robot.lFlip.setPosition(robot.lFlip.getPosition() - 0.01);
            }
            if(gamepad1.dpad_up)
            {
                robot.rFlip.setPosition(robot.rFlip.getPosition() + 0.01);
                robot.lFlip.setPosition(robot.lFlip.getPosition() + 0.01);
            }


            //Player 2 team
            if(gamepad2.a)
            {
                robot.cServo.setPosition(0.7);
            }
            /**
             *


            if(gamepad2.b)
            {
                robot.rightClaw.setPosition(1.0);
            }
            if(gamepad2.dpad_down)
            {
                robot.cam.setPosition(robot.cam.getPosition() - 0.02);
            }
            if(gamepad2.dpad_up)
            {
                robot.cam.setPosition(robot.cam.getPosition() + 0.02);
            }
             **/
            //right2 = gamepad2.left_stick_y;
            //armDrive(false,false );
            left2 = gamepad2.right_stick_y;
            elevatorDrive(false,false);
        }
            // Show the elapsed game time and wheel power.................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", right, left);
            telemetry.update();
        }

    void drive(boolean precise, boolean reverse) {
        double right_scaled = scaleMotor(right,precise);
        double left_scaled = scaleMotor(left,precise);

        if(reverse) {
            double temp = right_scaled;
            right_scaled = -left_scaled;
            left_scaled = -temp;
        }

        robot.rightDrive.setPower(right_scaled);
        robot.rightDrive2.setPower(right_scaled);

        robot.leftDrive.setPower(left_scaled);
        robot.leftDrive2.setPower(left_scaled);
    }
    double scaleMotor(double num, boolean precise) {
        if (num == 0.0)
            return 0.0;
        //For precision mode
        double[] scaleArray = {0.5, 0.75, 1.0};
        double[] preciseArray = {0.1, 0.2, 0.3};
        // get the corresponding index for the scaleInput array.
        int index = (int) (num * (scaleArray.length - 1));
        index = Math.abs(index);

        double scaled;
        if (precise)
            scaled = preciseArray[index];
        else
            scaled = scaleArray[index];
        if (num < 0.0)
            scaled = -scaled;

        return scaled;
    }
    void armDrive(boolean precise, boolean reverse) {
        double right_scaled = scaleMotor(right2,precise);
        robot.reel.setPower(right_scaled);
    }
    void elevatorDrive(boolean precise, boolean reverse) {
        double left_scaled = scaleMotor(left2,precise);
        robot.elevator.setPower(left_scaled);
    }
}
