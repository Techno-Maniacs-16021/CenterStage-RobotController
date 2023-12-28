package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import android.util.Log;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.bot.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;


@Config
@TeleOp
public class Rick_Driver_Mode extends OpMode
{
    /////////////////////////////////////////////
    ServoImplEx intake_arm,claw,claw_angler;
    CRServo left_aligner;
    DcMotorEx left_slides,right_slides,left_intake,right_intake;
    AnalogInput claw_Position, claw_Angle;
    RevColorSensorV3 pixelDetector;
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    /////////////////////////////////////////////
    private ElapsedTime loopTime = new ElapsedTime();
    private MecanumDrive drive;
    private AprilTagProcessor tagProcessor;
    private TfodProcessor tfodProcessor;
    public static double clawPosition,clawAngle,intakePower,speedMultiplier;
    boolean intaked,outaked,OVERIDE;
    public static int HIGH,MID;
    public static double zeroLimit,oneLimit,twoLimit;
    public static double p,i,d,f,Target;
    private PIDController Controller;
    public static int INTIAL_OFFSET,PIXEL_LAYER;
    public double slidePower;
    public static double distance = 0;
    /////////////////////////////////////////////
    @Override
    public void init(){
////////////////////////HARDWARE INIT////////////////
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        intake_arm = hardwareMap.get(ServoImplEx.class, "IA");
        claw = hardwareMap.get(ServoImplEx.class, "Claw");
        claw_angler = hardwareMap.get(ServoImplEx.class,"CA");

        left_aligner = hardwareMap.get(CRServo.class,"Aligner");

        claw_Position = hardwareMap.get(AnalogInput.class,"Claw Pos");
        claw_Angle = hardwareMap.get(AnalogInput.class,"Claw Angle");

        left_slides = hardwareMap.get(DcMotorEx.class,"LS");
        right_slides = hardwareMap.get(DcMotorEx.class,"RS");
        left_intake = hardwareMap.get(DcMotorEx.class,"LI");
        right_intake = hardwareMap.get(DcMotorEx.class,"RI");

        pixelDetector = hardwareMap.get(RevColorSensorV3.class,"PD");

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "led");
////////////////////////SET PWM RANGE////////////////

        intake_arm.setPwmRange(new PwmControl.PwmRange(510,2490));
        claw.setPwmRange(new PwmControl.PwmRange(510,2490));
        claw_angler.setPwmRange(new PwmControl.PwmRange(510,2490));
////////////////////////HARDWARE REVERSING///////////
        left_slides.setDirection(DcMotorSimple.Direction.REVERSE);
        right_intake.setDirection(DcMotorSimple.Direction.REVERSE);
////////////////////////MOTOR BRAKE BEHAVIOR/////////
        left_slides.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right_slides.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
////////////////////////ENCODER RESET////////////////
        left_slides.setMode(RUN_WITHOUT_ENCODER);
        right_slides.setMode(RUN_WITHOUT_ENCODER);
        left_intake.setMode(STOP_AND_RESET_ENCODER);
        left_intake.setMode(RUN_WITHOUT_ENCODER);
////////////////////////PID CONTROLLERS//////////////
        Controller = new PIDController(p,i,d);
////////////////////////DASHBOARD TELEMETRY//////////
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
////////////////////////STATUS UPDATE////////////////
        telemetry.addData("Status", "Initialized");
/////////////////////////////////////////////////////
        tagProcessor = new AprilTagProcessor.Builder().setDrawTagID(true).setDrawTagOutline(true).setDrawAxes(true).setDrawCubeProjection(true).build();
        tfodProcessor = new TfodProcessor.Builder().setModelAssetName("Model.tflite").setModelLabels(new String[]{"Blue Prop", "Red Prop"}).build();
        VisionPortal visionPortal = new VisionPortal.Builder().addProcessor(tfodProcessor).addProcessor(tagProcessor).setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")).setCameraResolution(new Size(640, 480)).setStreamFormat(VisionPortal.StreamFormat.MJPEG).build();

        p=0;i=0;d=0;f=0;Target = 0;speedMultiplier=1;
        INTIAL_OFFSET = 0;PIXEL_LAYER= 0;
        zeroLimit=80;oneLimit=66;twoLimit=55;
        intaked = false;

        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        blinkinLedDriver.setPattern(pattern);

    }
    @Override
    public void init_loop(){
        List<Recognition> currentRecognitions = tfodProcessor.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());
        for(Recognition recognition : currentRecognitions){
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", (recognition.getLeft() + recognition.getRight()) / 2, (recognition.getTop()  + recognition.getBottom()) / 2);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }
    }
    @Override
    public void start(){
        tfodProcessor.shutdown();
    }
    @Override
    public void loop(){
        //
        clawAngle=claw_Angle.getVoltage();
        // April Tags
        if (tagProcessor.getDetections().size() > 0){
            AprilTagDetection tag = tagProcessor.getDetections().get(0);
            telemetry.addData("x", tag.id);
            telemetry.addData("x", tag.ftcPose.x);
            telemetry.addData("y", tag.ftcPose.y);
            telemetry.addData("z", tag.ftcPose.z);
            telemetry.addData("roll", tag.ftcPose.roll);
            telemetry.addData("pitch", tag.ftcPose.pitch);
            telemetry.addData("yaw", tag.ftcPose.yaw);
        }
        telemetry.update();
    ////////////////////LED LOGIC///////////
        if(pixelDetector.getDistance(DistanceUnit.MM)>twoLimit&&pixelDetector.getDistance(DistanceUnit.MM)<oneLimit){
            pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            blinkinLedDriver.setPattern(pattern);
        }
        else if(pixelDetector.getDistance(DistanceUnit.MM)>oneLimit&&pixelDetector.getDistance(DistanceUnit.MM)<zeroLimit){
            pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
            blinkinLedDriver.setPattern(pattern);
        }
        else if(pixelDetector.getDistance(DistanceUnit.MM)>zeroLimit){
            pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
            blinkinLedDriver.setPattern(pattern);
        }
        else{
            pattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_RED;
            blinkinLedDriver.setPattern(pattern);
        }
////////////////////////SLIDES PID//////////
        Controller.setPID(p,i,d);
        int Pos = (right_slides.getCurrentPosition()+left_slides.getCurrentPosition())/2;
        double PID = Controller.calculate(Pos,Target);
        double Power = PID+f;
            //left_slides.setPower(Power);
            //right_slides.setPower(Power);
//INTAKE
        if(gamepad1.a){
            left_intake.setPower(1);
            right_intake.setPower(1);
            left_aligner.setPower(1);
        }
        else if (gamepad1.b){
            left_intake.setPower(-1);
            right_intake.setPower(-1);
            left_aligner.setPower(0);
        }
        else {
            left_intake.setPower(0);
            right_intake.setPower(0);
            left_aligner.setPower(0);
        }
        if(gamepad1.dpad_up)intake_arm.setPosition(1);
        else if(gamepad1.dpad_down)intake_arm.setPosition(0.15);
//Slides
        left_slides.setPower(slidePower);
        right_slides.setPower(slidePower);
        if(gamepad1.right_trigger!=0) slidePower = gamepad1.right_trigger;
        else if(gamepad1.left_trigger!=0) slidePower = -gamepad1.left_trigger;
        else slidePower = 0;
//Outtake
        if(gamepad1.left_bumper){
        claw_angler.setPosition(1);
        claw.setPosition(0.5);
        }
        else if(gamepad1.right_bumper)claw_angler.setPosition(0);
        else if(gamepad1.dpad_left) claw_angler.setPosition(0.3);
        if(gamepad1.x)claw.setPosition(1);
        else if(gamepad1.y)claw.setPosition(0);
        else if(gamepad1.dpad_right){
            if(clawAngle>2.19){
                claw_angler.setPosition(0.05);
            }
            else if(clawAngle<2.18){
                claw_angler.setPosition(0);
            }
            claw.setPosition(0.5);
        }
////////////////////////DRIVE LOGIC//////////////////

        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y ,
                        -gamepad1.left_stick_x
                ),
                -gamepad1.right_stick_x
        ));

        drive.updatePoseEstimate();
        telemetry.addLine("Running: TechnoManiacs CenterStage Operating System - V0.1");

////////////////////////TELEMETRY////////////////////
        telemetry.addData("loop time: ",loopTime.time());
        telemetry.addData("Claw Position: ",clawPosition);
        telemetry.addData("Claw Angle: ",clawAngle);
        telemetry.addData("Slide Speed: ",Power);
        telemetry.addData("distance in MM:",pixelDetector.getDistance(DistanceUnit.MM));
        distance=pixelDetector.getDistance(DistanceUnit.MM);
        Log.i("pixelDistance", ""+pixelDetector.getDistance(DistanceUnit.MM));
        loopTime.reset();
        telemetry.update();
    }
    @Override
    public void stop(){

    }

}
