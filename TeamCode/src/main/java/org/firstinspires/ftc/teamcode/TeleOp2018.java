
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name="TeleOP2018", group="Linear Opmode")
public class TeleOp2018 extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    static double right;
    static double left;
    static double right2;
    static double left2;
    RobotConfig2018 robot = new RobotConfig2018();
    @Override
    public void runOpMode() {
        // Save reference to Hardware map
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.init(hardwareMap);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            int mode = 1;
            if (gamepad1.right_stick_button && mode == 1) {
                mode++;
                sleep(100);
            } else if (gamepad1.right_stick_button && mode == 2) {
                mode--;
                sleep(100);
            }
            //Tank Drive
            if (mode == 2) {
                right = gamepad1.right_stick_y;
                left = gamepad1.left_stick_y;
                boolean precisionMode = false;
                if (gamepad1.left_trigger != 0)
                    precisionMode = true;
                boolean reverseMode = false;
                if (gamepad1.right_trigger != 0)
                    reverseMode = true;
                drive(precisionMode, reverseMode);
            } else if (mode == 1) {
                //Mech Drive
                telemetry.addData("Driving", "true");
                telemetry.update();
               // mecanumDrive_Cartesian(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
                double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
                double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
                double rightX = gamepad1.right_stick_x;
                final double v1 = r * Math.cos(robotAngle) + rightX;
                final double v2 = r * Math.sin(robotAngle) - rightX;
                final double v3 = r * Math.sin(robotAngle) + rightX;
                final double v4 = r * Math.cos(robotAngle) - rightX;

                robot.leftDrive.setPower(v1);
                robot.rightDrive.setPower(v2);
                robot.leftDrive2.setPower(v3);
                robot.rightDrive2.setPower(v4);
            }
            //Controls linear actuator
            right2 = gamepad2.right_stick_y;
            armDrive(false,false);
            // Show the elapsed game time and wheel power.................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", right, left);
            telemetry.update();
            left2 = gamepad2.left_stick_y;
            screwDrive(false,false);
            if(gamepad2.dpad_up)
            {
                robot.climb.setPower(-1);
            }
            else if(gamepad2.dpad_down)
            {
                robot.climb.setPower(1);
            }
            else if(gamepad2.dpad_right)
            {
                robot.climb.setPower(0);
            }
            else if(gamepad2.a)
            {
                robot.sServo.setPosition(90);
            }
        }
    }

    void drive(boolean precise, boolean reverse) {
        double right_scaled = scaleMotor(right,precise);
        double left_scaled = scaleMotor(left,precise);

        if(reverse) {
            double temp = right_scaled;
            right_scaled = -left_scaled;
            left_scaled = -temp;
        }
        //Set power for motors
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
    public void mecanumDrive_Cartesian(double x, double y, double rotation) {
        double wheelSpeeds[] = new double[4];

        wheelSpeeds[0] = x + y + rotation; //left wheel
        wheelSpeeds[1] = -x + y - rotation;//right wheel
        wheelSpeeds[2] = -x + y + rotation;//left wheel 2
        wheelSpeeds[3] = x + y - rotation;//right wheel 2

        normalize(wheelSpeeds);

        robot.leftDrive.setPower(wheelSpeeds[0]);
        robot.rightDrive.setPower(wheelSpeeds[1]);
        robot.leftDrive2.setPower(wheelSpeeds[2]);
        robot.rightDrive2.setPower(wheelSpeeds[3]);
    }


    private void normalize(double[] wheelSpeeds)
    {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);


        for (int i = 1; i < wheelSpeeds.length; i++)
        {
            double magnitude = Math.abs(wheelSpeeds[i]);

            if (magnitude > maxMagnitude)
            {
                maxMagnitude = magnitude;
            }
        }

        if (maxMagnitude > 1.0)
        {
            for (int i = 0; i < wheelSpeeds.length; i++)
            {
                wheelSpeeds[i] /= maxMagnitude;
            }
        }
    }
    void armDrive(boolean precise, boolean reverse) {
        double right_scaled = scaleMotor(right2,precise);
        robot.reel.setPower(right_scaled);
    }

    void screwDrive(boolean precise,boolean reverse)
    {
        double right_scaled = scaleMotor(left2,precise);
        robot.screw.setPower(right_scaled);
    }





    // New Mech code
    /**
    public void test(){
        double Speed = -gamepad1.left_stick_y;
        double Turn = gamepad1.left_stick_x;
        double Strafe = gamepad1.right_stick_x;
        double MAX_SPEED = 1.0;
        //holonomic(Speed, Turn, Strafe, MAX_SPEED );
    }

    public void holonomic(double Speed, double Turn, double Strafe, double MAX_SPEED){

//      Left Front = +Speed + Turn - Strafe      Right Front = +Speed - Turn + Strafe
//      Left Rear  = +Speed + Turn + Strafe      Right Rear  = +Speed - Turn - Strafe

        double Magnitude = abs(Speed) + abs(Turn) + abs(Strafe);
        Magnitude = (Magnitude > 1) ? Magnitude : 1; //Set scaling to keep -1,+1 range

        leftFrontMotor.setPower(scale((scaleInput(Speed) + scaleInput(Turn) - scaleInput(Strafe)),
                -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
        if (leftRearMotor != null) {
            leftRearMotor.setPower(scale((scaleInput(Speed) + scaleInput(Turn) + scaleInput(Strafe)),
                    -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
        }
        rightFrontMotor.setPower(scale((scaleInput(Speed) - scaleInput(Turn) + scaleInput(Strafe)),
                -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
        if (rightRearMotor != null) {
            rightRearMotor.setPower(scale((scaleInput(Speed) - scaleInput(Turn) - scaleInput(Strafe)),
                    -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
        }
    }
     **/
}
