package org.firstinspires.ftc.teamcode.auto;

import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.Detector;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@TeleOp
public class DetectionDemo extends LinearOpMode {

    @Override
    public void runOpMode() {
        TfodProcessor tfodProcessor = TfodProcessor.easyCreateWithDefaults();
        VisionPortal visionPortal = new VisionPortal.Builder().addProcessor(tfodProcessor).setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")).setCameraResolution(new Size(640, 480)).setStreamFormat(VisionPortal.StreamFormat.MJPEG).build();
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            List<Recognition> currentRecognitions = tfodProcessor.getRecognitions();
            telemetry.addData("# Objects Detected", currentRecognitions.size());
            for(Recognition recognition : currentRecognitions){
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position", "%.0f / %.0f", (recognition.getLeft() + recognition.getRight()) / 2, (recognition.getTop()  + recognition.getBottom()) / 2);
                telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            }
            telemetry.update();
        }
    }
}
