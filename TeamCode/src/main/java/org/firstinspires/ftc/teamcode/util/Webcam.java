package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class Webcam {
    public OpenCvWebcam webcam;
    private OpenCvPipeline pipeline;

    public Webcam(String webcamNameStr, HardwareMap hardwareMap){
        WebcamName webcamName = hardwareMap.get(WebcamName.class, webcamNameStr);
        //init cameramonitorview
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //initialize you webcam object
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        //initialize our pipeline
        //pipeline = new SleeveDetection();
        //set the pipeline accordingly
        webcam.setPipeline(pipeline);
        //to view the camera on the scrcpy
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }

            public void stopStream(){
                webcam.stopStreaming();
            }
        });

    }

    /*public SleeveDetection.ParkingPosition getPosition() {
        return ((SleeveDetection) pipeline).getPosition();
    }*/
}
