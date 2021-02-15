package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

public class StudCam  {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    public double ringHeight;
    public int numObjects;
    private static final String VUFORIA_KEY =
            "AdUb35L/////AAABmRvLTWKDkUCtiR+XIKPqagcuwUCwyQjRG79vGuoovT1yInddk+tLIL7cRoa4nXq5QtV/Db/Vr2oAKcK4N6b+P36dRXQo66pP8FueLxMGRlSgFLVvV5/jPpnvwxA4NNXBtNhhGDARdYjDfF90SegvSr8uCOrvJpuXsV6rtgpovED/q+IMSY6KUNS+K16lgIX2Ox+TuMfVlnLVMP4BXdFpjb0FJ9c1cj55GE3c779f4EOznTqwy6r08IBDnpcacOfymkwAi2Lsbe5WJJ0XdrhrqC94UEVx27lISzZtHDn0OjRnmQWcEH+vNyQ33s3V8yNftiUx0m+aCYtqINaeqlXBBzMvzwJFmyF8IW7B8b3ULfpu";

    private VuforiaLocalizer vuforia;


    private TFObjectDetector tfod;

    public void init(HardwareMap hardwareMap) {

        initVuforia();
        initTfod(hardwareMap);

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.2, 16.0/9.0);
        }



    }

    public void stopAll() {
        if (tfod != null) {
            tfod.shutdown();
            tfod.deactivate();

        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;
        parameters.fillCameraMonitorViewParent = true;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.6f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

    }



     String determineZone(){
         List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
         numObjects = updatedRecognitions.size();

        if (numObjects == 0){
            return "A";
        }
        ringHeight = updatedRecognitions.get(0).getHeight();
        if (ringHeight >95.0){
            return"C";
        }
        return "B";
    }







}