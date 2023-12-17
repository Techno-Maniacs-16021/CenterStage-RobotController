package org.firstinspires.ftc.teamcode.auto.redpixel;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

/*
 * This is a simple routine to test turning capabilities.
 */
@Autonomous
@Config
public class RPBasic extends LinearOpMode {

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


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-36,-64,3 * Math.PI / 2));

        intake_arm = hardwareMap.get(ServoImplEx.class, "IA");
        claw = hardwareMap.get(ServoImplEx.class, "Claw");
        claw_angler = hardwareMap.get(ServoImplEx.class,"CA");

        aligner = hardwareMap.get(CRServo.class,"Aligner");

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
        left_slides.setMode(STOP_AND_RESET_ENCODER);
        right_slides.setMode(STOP_AND_RESET_ENCODER);
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
        p=0.0035;i=0;d=0.0001;f=0.075;Target = 0;speedMultiplier=1;
        INTIAL_OFFSET = 800;PIXEL_LAYER= 100;ALLOWED_ERROR=30;INTAKE_OFFSET=300;
        zeroLimit=80;oneLimit=66;twoLimit=55;
        intaked = false; intakeReady=false; outtaked = false; actionInit = false;
        claw_angler.setPosition(1);
        claw.setPosition(1);

        waitForStart();

        clawAngle=claw_Angle.getVoltage();
        clawPosition=claw_Position.getVoltage();

        while (opModeIsActive()) {
            Actions.runBlocking(drive.actionBuilder(new Pose2d(-36, -62, 3 * Math.PI / 2))
                    .splineToConstantHeading(new Vector2d(-36, -36), 3 * Math.PI / 2)
                    .turn(-Math.PI / 2)
                    .build());

            //raise slides
            Target = 1000;
            while(getError(((right_slides.getCurrentPosition() + left_slides.getCurrentPosition()) / 2),Target)>=ALLOWED_ERROR&&!isStopRequested()) {
                Controller.setPID(p, i, d);
                double PID = Controller.calculate(((right_slides.getCurrentPosition() + left_slides.getCurrentPosition()) / 2.0), Target);
                double Power = PID + f;
                left_slides.setPower(Power);
                right_slides.setPower(Power);
            }
            //angle claw out
            claw_angler.setPosition(0.25);
            sleep(500);
            //lower slides
            Target = 0;
            while(getError(((right_slides.getCurrentPosition() + left_slides.getCurrentPosition()) / 2),Target)>=ALLOWED_ERROR&&!isStopRequested()) {
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
            //raise slides
            Target = 1000;
            while(getError(((right_slides.getCurrentPosition() + left_slides.getCurrentPosition()) / 2),Target)>=ALLOWED_ERROR&&!isStopRequested()) {
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
            Target = 0;
            while(getError(((right_slides.getCurrentPosition() + left_slides.getCurrentPosition()) / 2),Target)>=ALLOWED_ERROR&&!isStopRequested()) {
                Controller.setPID(p, i, d);
                double PID = Controller.calculate(((right_slides.getCurrentPosition() + left_slides.getCurrentPosition()) / 2.0), Target);
                double Power = PID + f;
                left_slides.setPower(Power);
                right_slides.setPower(Power);
            }

            Actions.runBlocking(drive.actionBuilder(new Pose2d(-36, -36, Math.PI))
                    .strafeTo(new Vector2d(-36, -12))
                    .splineToConstantHeading(new Vector2d(36, -12), Math.PI)
                    .strafeTo(new Vector2d(36, -36))
                    .splineToConstantHeading(new Vector2d(51, -36), Math.PI)
                    .build());

            Actions.runBlocking(drive.actionBuilder(new Pose2d(51, -36, Math.PI))
                    .strafeTo(new Vector2d(51, -60))
                    .splineToConstantHeading(new Vector2d(60, -60), Math.PI)
                    .build());
            
            requestOpModeStop();
        }
    }

    public static int getError(int current, double target){
        int error = Math.abs((int)target-current);
        return error;
    }

}
