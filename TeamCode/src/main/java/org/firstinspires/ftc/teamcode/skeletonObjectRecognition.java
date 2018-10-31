package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;


@Autonomous(name="ObjectRecognition", group ="Pushbot")
public class skeletonObjectRecognition extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
//Initiliazes the robot
        GoldAlignDetector goldAlignDetector = new GoldAlignDetector();
        goldAlignDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        goldAlignDetector.useDefaults();

        goldAlignDetector.enable();
        while(opModeIsActive())
        {
            telemetry.addData("IsAligned" , goldAlignDetector.getAligned()); // Is the bot aligned with the gold mineral
            telemetry.addData("X Pos" , goldAlignDetector.getXPosition()); // Gold X pos.


            if(!goldAlignDetector.getAligned()){
                //Not aligned, keep going
                telemetry.addLine("NOOOOO");
                telemetry.update();
            }else{
                //Aligned!
                telemetry.addLine("YASSSS");
                telemetry.update();

            }
        }
    }
}
