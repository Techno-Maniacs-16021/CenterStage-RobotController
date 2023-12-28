package org.firstinspires.ftc.teamcode.auto.bluepixel;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.bot.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/*
 * This is a simple routine to test turning capabilities.
 */
@Autonomous
@Config
public class BPixel extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    private static final String TFOD_MODEL_ASSET = "blueCloseDepthModel.tflite";
    private static final String[] LABELS = {
            "Blue Prop",
    };
    ServoImplEx intake_arm,claw,claw_angler;
    CRServo aligner;
    DcMotorEx left_slides,right_slides,left_intake,right_intake;
    AnalogInput claw_Position, claw_Angle;
    RevColorSensorV3 pixelDetector;
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    private ElapsedTime loopTime = new ElapsedTime();
    private AprilTagProcessor tagProcessor;
    private TfodProcessor tfodProcessor;
    public static double clawPosition,clawAngle,intakePower,speedMultiplier;
    boolean intaked,outtaked,OVERIDE, intakeReady, actionInit;
    public static int HIGH,MID;
    public static double zeroLimit,oneLimit,twoLimit;
    public static double p,i,d,f,Target;
    private PIDController Controller;
    public static int INTIAL_OFFSET,PIXEL_LAYER, ALLOWED_ERROR, INTAKE_OFFSET;
    public double slidePower;
    public static double distance = 0;

    public static boolean straight, left, right;


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-36, 62,  Math.PI / 2));

        intake_arm = hardwareMap.get(ServoImplEx.class, "IA");
        claw = hardwareMap.get(ServoImplEx.class, "Claw");
        claw_angler = hardwareMap.get(ServoImplEx.class, "CA");

        aligner = hardwareMap.get(CRServo.class, "Aligner");

        claw_Position = hardwareMap.get(AnalogInput.class, "Claw Pos");
        claw_Angle = hardwareMap.get(AnalogInput.class, "Claw Angle");

        left_slides = hardwareMap.get(DcMotorEx.class, "LS");
        right_slides = hardwareMap.get(DcMotorEx.class, "RS");
        left_intake = hardwareMap.get(DcMotorEx.class, "LI");
        right_intake = hardwareMap.get(DcMotorEx.class, "RI");

        pixelDetector = hardwareMap.get(RevColorSensorV3.class, "PD");

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "led");
////////////////////////SET PWM RANGE////////////////
        intake_arm.setPwmRange(new PwmControl.PwmRange(510, 2490));
        claw.setPwmRange(new PwmControl.PwmRange(510, 2490));
        claw_angler.setPwmRange(new PwmControl.PwmRange(510, 2490));
////////////////////////HARDWARE REVERSING///////////
        left_slides.setDirection(DcMotorSimple.Direction.REVERSE);
        right_intake.setDirection(DcMotorSimple.Direction.REVERSE);
////////////////////////MOTOR BRAKE BEHAVIOR/////////
        left_slides.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right_slides.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
////////////////////////ENCODER RESET////////////////
        left_slides.setMode(STOP_AND_RESET_ENCODER);
        right_slides.setMode(STOP_AND_RESET_ENCODER);
        left_slides.setMode(RUN_WITHOUT_ENCODER);
        right_slides.setMode(RUN_WITHOUT_ENCODER);
        left_intake.setMode(STOP_AND_RESET_ENCODER);
        left_intake.setMode(RUN_WITHOUT_ENCODER);
////////////////////////PID CONTROLLERS//////////////
        Controller = new PIDController(p, i, d);
////////////////////////DASHBOARD TELEMETRY//////////
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
////////////////////////STATUS UPDATE////////////////
        telemetry.addData("Status", "Initialized");
/////////////////////////////////////////////////////
        p = 0.0035;
        i = 0;
        d = 0.0001;
        f = 0.075;
        Target = 0;
        speedMultiplier = 1;
        INTIAL_OFFSET = 800;
        PIXEL_LAYER = 100;
        ALLOWED_ERROR = 30;
        INTAKE_OFFSET = 300;
        zeroLimit = 80;
        oneLimit = 66;
        twoLimit = 55;
        intaked = false;
        intakeReady = false;
        outtaked = false;
        actionInit = false;
        claw_angler.setPosition(1);
        claw.setPosition(1);
        intake_arm.setPosition(1);
        straight = true;
        left = false;
        right = false;

        initTfod();

        waitForStart();

        clawAngle = claw_Angle.getVoltage();
        clawPosition = claw_Position.getVoltage();

        while (opModeIsActive()) {
            Actions.runBlocking(drive.actionBuilder(new Pose2d(-36, 62, Math.PI / 2))
                        .splineToConstantHeading(new Vector2d(-36, 44), Math.PI / 2)
                    .build());
            intake_arm.setPosition(1);
            double startTime = time;
            boolean found = false;

            while(time - startTime < 2 && !found && opModeIsActive()) {
                telemetry.addData("time: ", time);
                found = objectDetected();
                telemetry.addData("found: ", found);
                telemetry.update();
            }
            if(found){
                straight = true;
                Actions.runBlocking(drive.actionBuilder(new Pose2d(-36,44, Math.PI / 2))
                        .turn(Math.PI/2)
                        .strafeTo(new Vector2d(-36,14))
                        .turn(Math.PI/2)
                        .build());
                releaseOnePixel();
                Actions.runBlocking(drive.actionBuilder(new Pose2d(-36,14, 3 * Math.PI / 2))
                        .turn(-Math.PI / 2)
                        .build());
            }else{

                Actions.runBlocking(drive.actionBuilder(new Pose2d(-36,44,Math.PI / 2))
                        .turn(-Math.PI / 4)
                        .build());
                sleep(500);
                startTime = time;
                while(time - startTime < 2 && !found && opModeIsActive()) {
                    telemetry.addData("time: ", time);
                    found = objectDetected();
                    telemetry.addData("found: ", found);
                    telemetry.update();
                }
                if(found){
                    right = true;
                    Actions.runBlocking(drive.actionBuilder(new Pose2d(-36,36,Math.PI / 4))
                            .turn(-Math.PI/4)
                            .strafeTo(new Vector2d(-38,34))
                            .build());
                    releaseOnePixel();
                    Actions.runBlocking(drive.actionBuilder(new Pose2d(-38,34,0))
                            .strafeTo(new Vector2d(-36,12))
                            .turn(-Math.PI)
                            .build());
                }else{
                    left = true;
                    Actions.runBlocking(drive.actionBuilder(new Pose2d(-36,36,Math.PI / 4))
                            .turn(3 * Math.PI/4)
                            .strafeTo(new Vector2d(-34,34))
                            .build());
                    releaseOnePixel();
                    Actions.runBlocking(drive.actionBuilder(new Pose2d(-34,34, Math.PI))
                            .strafeTo(new Vector2d(-36,12))
                            .build());
                }
            }
            visionPortal.stopStreaming();
//            visionPortal.close();
            //raise slides

            //raise slides
            Target = 1000;
            while (getError(((right_slides.getCurrentPosition() + left_slides.getCurrentPosition()) / 2), Target) >= ALLOWED_ERROR && opModeIsActive()) {
                Controller.setPID(p, i, d);
                double PID = Controller.calculate(((right_slides.getCurrentPosition() + left_slides.getCurrentPosition()) / 2.0), Target);
                double Power = PID + f;
                left_slides.setPower(Power);
                right_slides.setPower(Power);
            }
            //angle claw in
            claw_angler.setPosition(1);
            sleep(500);
            //lower slides
            Target = 150;
            while (getError(((right_slides.getCurrentPosition() + left_slides.getCurrentPosition()) / 2), Target) >= ALLOWED_ERROR && opModeIsActive()) {
                Controller.setPID(p, i, d);
                double PID = Controller.calculate(((right_slides.getCurrentPosition() + left_slides.getCurrentPosition()) / 2.0), Target);
                double Power = PID + f;
                left_slides.setPower(Power);
                right_slides.setPower(Power);
            }

            Actions.runBlocking(drive.actionBuilder(new Pose2d(-36, 12, Math.PI))
                    .splineToConstantHeading(new Vector2d(36, 12), Math.PI)
                    .strafeTo(new Vector2d(36, 36))
                    .splineToConstantHeading(new Vector2d(51 + (right || left ? 2 : 0), 38 + (right ? -6 : left ? 6 : 0)), Math.PI)
                    .build());

            //raise slides
            Target = 1000;
            while (getError(((right_slides.getCurrentPosition() + left_slides.getCurrentPosition()) / 2), Target) >= ALLOWED_ERROR && opModeIsActive()) {
                Controller.setPID(p, i, d);
                double PID = Controller.calculate(((right_slides.getCurrentPosition() + left_slides.getCurrentPosition()) / 2.0), Target);
                double Power = PID + f;
                left_slides.setPower(Power);
                right_slides.setPower(Power);
            }
            //claw out
            claw_angler.setPosition(0.05);
            sleep(500);
            claw.setPosition(0);
            sleep(500);
            claw.setPosition(0.5);
            sleep(500);
            //claw in
            claw_angler.setPosition(1);
            sleep(500);
            //lower slides
            Target = 150;
            while (getError(((right_slides.getCurrentPosition() + left_slides.getCurrentPosition()) / 2), Target) >= ALLOWED_ERROR && opModeIsActive()) {
                Controller.setPID(p, i, d);
                double PID = Controller.calculate(((right_slides.getCurrentPosition() + left_slides.getCurrentPosition()) / 2.0), Target);
                double Power = PID + f;
                left_slides.setPower(Power);
                right_slides.setPower(Power);
            }

            Actions.runBlocking(drive.actionBuilder(new Pose2d(51 + (right || left ? 2 : 0), 38 + (right ? -6 : left ? 6 : 0), Math.PI))
                    .strafeTo(new Vector2d(51 + (right || left ? 2 : 0), 12))
                    .splineToConstantHeading(new Vector2d(60, 12), Math.PI)
                    .build());
            sleep(1000);
            requestOpModeStop();
        }
    }

    public static int getError(int current, double target){
        int error = Math.abs((int)target-current);
        return error;
    }

    public boolean setPositionOfSlides(double Target){
        return getError(((right_slides.getCurrentPosition() + left_slides.getCurrentPosition()) / 2), Target) >= ALLOWED_ERROR && opModeIsActive();
    }
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)

                //.setModelFileName(TFOD_MODEL_FILE)

                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }
    private boolean objectDetected() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());
        boolean atLeastOne = false;
        for(Recognition rec : currentRecognitions){
            atLeastOne = atLeastOne || rec.getWidth() * 2 < rec.getImageWidth();
            Log.i("detections - width", rec.getWidth() + "/" + rec.getImageWidth());
        }
        Log.i("detections", currentRecognitions.size() + "");
        return currentRecognitions.size() > 0 && atLeastOne;

    }

    private void releaseOnePixel(){
        Target = 1000;
        while (setPositionOfSlides(Target)) {
            Controller.setPID(p, i, d);
            double PID = Controller.calculate(((right_slides.getCurrentPosition() + left_slides.getCurrentPosition()) / 2.0), Target);
            double Power = PID + f;
            left_slides.setPower(Power);
            right_slides.setPower(Power);
        }
        //angle claw out
        claw_angler.setPosition(0.3);
        sleep(500);
        //lower slides
        Target = 150;
        while(getError(((right_slides.getCurrentPosition() + left_slides.getCurrentPosition()) / 2),Target)>=ALLOWED_ERROR&&opModeIsActive()) {
            Controller.setPID(p, i, d);
            double PID = Controller.calculate(((right_slides.getCurrentPosition() + left_slides.getCurrentPosition()) / 2.0), Target);
            double Power = PID + f;
            left_slides.setPower(Power);
            right_slides.setPower(Power);
        }
        //release 1 pixel
        sleep(500);
        claw.setPosition(0);
        sleep(500);
    }

}
