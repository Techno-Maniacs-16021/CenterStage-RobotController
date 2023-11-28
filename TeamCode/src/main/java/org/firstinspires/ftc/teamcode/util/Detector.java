package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.tfrec.classification.Classifier;

import java.util.List;

public class Detector implements Runnable {
    Telemetry telemetry;
    private org.firstinspires.ftc.teamcode.tfrec.Detector tfDetector = null;
    private HardwareMap hardwareMap;

    private boolean isRunning = true;


    private String modelFileName = "model_unquant.tflite";
    private String labelFileName = "labels.txt";
    private static Classifier.Model MODEl_TYPE = Classifier.Model.FLOAT_EFFICIENTNET;
    private static final String LABEL_A = "Center";
    private static final String LABEL_B = "Gamble";

    private String result = LABEL_B; //just a default value.

    private LinearOpMode caller = null;



    public Detector(HardwareMap hMap, LinearOpMode caller, Telemetry t) throws Exception {
        hardwareMap = hMap;
        telemetry = t;
        initDetector();
        activateDetector();
        this.caller = caller;

    }

    public Detector(HardwareMap hMap, LinearOpMode caller, Telemetry t, String model, String labels) throws Exception {
        hardwareMap = hMap;
        telemetry = t;
        setModelFileName(model);
        setLabelFileName(labels);
        initDetector();
        activateDetector();
        this.caller = caller;
    }




    public void detectObjectThread() {

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (isRunning && caller.opModeIsActive()) {
            if (tfDetector != null) {
                List<Classifier.Recognition> results = tfDetector.getLastResults();
                if (results == null || results.size() == 0) {
                    telemetry.addData("Nada", "No results");
                } else {
                    for (Classifier.Recognition r : results) {
                        if (r.getConfidence() >= 0.8) {
                            telemetry.addData("PrintZone", r.getTitle());
                            if(r.getTitle().contains(LABEL_B)){
                                this.result = LABEL_B;
                            }
                            else if(r.getTitle().contains(LABEL_A)){
                                this.result = LABEL_A;
                            }
                        }
                    }
                }
            }
            telemetry.update();
        }

    }


    public void initDetector() throws Exception {
        tfDetector = new org.firstinspires.ftc.teamcode.tfrec.Detector(MODEl_TYPE, getModelFileName(), getLabelFileName(), hardwareMap.appContext, telemetry);
    }

    protected void activateDetector() throws Exception {
        if (tfDetector != null) {
            tfDetector.activate();
        }
        telemetry.addData("Info", "TF Activated");
    }


    public void stopDetection() {
        stopThread();
        if (tfDetector != null) {
            tfDetector.stopProcessing();
        }
        tfDetector = null;
    }

    public void stopThread() {
        isRunning = false;
    }

    @Override
    public void run() {
        while(isRunning) {
            detectObjectThread();
        }
    }


    public String getModelFileName() {
        return modelFileName;
    }

    public void setModelFileName(String modelFileName) {
        this.modelFileName = modelFileName;
    }

    public String getLabelFileName() {
        return labelFileName;
    }

    public void setLabelFileName(String labelFileName) {
        this.labelFileName = labelFileName;
    }

    public String getResult() {
        return result;
    }

    public void setResult(String result) {
        this.result = result;
    }
}
