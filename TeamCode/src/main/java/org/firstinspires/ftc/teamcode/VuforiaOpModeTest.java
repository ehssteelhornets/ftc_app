package org.firstinspires.ftc.teamcode;
import android.opengl.GLES20;
import android.opengl.GLSurfaceView;
import android.opengl.Matrix;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.DataSet;
import com.vuforia.Matrix44F;
import com.vuforia.ObjectTarget;
import com.vuforia.ObjectTargetResult;
import com.vuforia.ObjectTracker;
import com.vuforia.Renderer;
import com.vuforia.RendererHelper;
import com.vuforia.STORAGE_TYPE;
import com.vuforia.State;
import com.vuforia.Tool;
import com.vuforia.Trackable;
import com.vuforia.TrackableResult;
import com.vuforia.TrackerManager;
import com.vuforia.VIDEO_BACKGROUND_REFLECTION;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
@TeleOp(name="Concept: Vuforia Object Recognition", group ="Concept")
public class VuforiaOpModeTest extends LinearOpMode{
    VuforiaLocalizer vuforia;
    private static final String VUFORIA_KEY = "AVBEWl3/////AAAAGbcR3zLzokcgvluieL+X7sUZchFxSNixRNcAWxR0bP9U+U43LsdX44KA/uRn41WeKGbwe4gTQcOdJve3abc+3SFCDWjM5NdDbjZkEutO2JcIigwn7jIU41jL6sXmCekHzJ7tW8F2B1JfISG6WP9KpbcD9F9BfMHnvBljUyT8nLP89/pqN0r8Zy2L5n9avC/LchzRCsMnvalZKZyYJkmlfNS8o4lKSOGzP2iEWx5a5J02jJiAwgPmsIjGKBWpUdwqB4fRlLLNxUMXKtRBIMEaLPn1+tdjMIIQX/fdf7q50MIWPdTdDdKJlbVHCiGrQa46ad5SA2+hFfsCglv8GW30Peuom9O5lGOWzjxSdcAv/H2W";
    // Initialize with invalid value:
    private int mProgressValue = -1;
    private boolean targetVisible = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static DataSet mCurrentDataset;
    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        //Get our data
        /**
        VuforiaTrackables targets = this.vuforia.loadTrackablesFromAsset("GameObjects2018-2019_OT");
        VuforiaTrackable ball = targets.get(0);
        ball.setName("Ball");
        VuforiaTrackable cube = targets.get(1);
        ball.setName("Cube");
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targets);
         **/
        //New data obtaining
        TrackerManager tManager = TrackerManager.getInstance();
        ObjectTracker objectTracker = (ObjectTracker) tManager
                .getTracker(ObjectTracker.getClassType());
        if (mCurrentDataset == null)
            mCurrentDataset = objectTracker.createDataSet();

        if (!mCurrentDataset.load("GameObjects2018-2019_OT.xml",
                STORAGE_TYPE.STORAGE_APPRESOURCE))
        {
            telemetry.addData("We are", "up shit creek without a paddle");
            telemetry.update();
        }
        objectTracker.activateDataSet(mCurrentDataset);
        //New vuforia code

        //Wir warten ewig.
        waitForStart();
        //targets.activate();
        while(opModeIsActive()) {
            int numTrackables = mCurrentDataset.getNumTrackables();
            for (int count = 0; count < numTrackables; count++) {
                Trackable trackable = mCurrentDataset.getTrackable(count);
                if (true) {
                    trackable.startExtendedTracking();
                }
                String name = "Current Dataset : " + trackable.getName();
                trackable.setUserData(name);

                telemetry.update();

                // did we find any trackables this frame?
                State state = TrackerManager.getInstance().getStateUpdater().updateState();
                //loop is never running, what is state?
                for (int tIdx = 0; tIdx < state.getNumTrackableResults(); tIdx++) {
                    telemetry.addData("Searching for:", "Objects");
                    TrackableResult result = state.getTrackableResult(tIdx);
                    Trackable trackable2 = result.getTrackable();
                    telemetry.addData("Item found", trackable2.getName());
                    telemetry.update();
                    if (!result.isOfType(ObjectTargetResult.getClassType()))
                        continue;
                    ObjectTarget objectTarget = (ObjectTarget) trackable2;
                }
            }
            }
        }
    // The render function.
    /**
    private void renderFrame()
    {

        //State state = mRenderer.begin();

        // did we find any trackables this frame?
        for (int tIdx = 0; tIdx < state.getNumTrackableResults(); tIdx++)
        {
            TrackableResult result = state.getTrackableResult(tIdx);
            Trackable trackable = result.getTrackable();
            telemetry.addData("Item found" , trackable.getName());
            if (!result.isOfType(ObjectTargetResult.getClassType()))
                continue;
            ObjectTarget objectTarget = (ObjectTarget) trackable;
        }
    }
     **/
}
