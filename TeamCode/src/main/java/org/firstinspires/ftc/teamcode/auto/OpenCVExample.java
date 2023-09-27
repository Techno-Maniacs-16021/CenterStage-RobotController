package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Webcam;

@TeleOp
public class OpenCVExample extends OpMode {
    /*
     Declare the following variables.
     */
    Webcam webcam;
    @Override
    public void init() {
        webcam = new Webcam("webcam", hardwareMap);
    }

    @Override
    public void loop() {
        //very important step do not skip!
        telemetry.addData("Status", "Violating...");
        //telemetry.addData("Go To", webcam.getPosition());
        telemetry.update();
    }

}

