package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;


@Config
@TeleOp
public class Driver_Mode extends OpMode
{
    /////////////////////////////////////////////
    ServoImplEx intake_arm,claw,claw_angler;
    CRServo aligner;
    DcMotorEx left_slides,right_slides,left_intake,right_intake;
    AnalogInput claw_Position, claw_Angle;
    /////////////////////////////////////////////
    private ElapsedTime loopTime = new ElapsedTime();
    private MecanumDrive drive;
    public static double clawPosition,clawAngle,speedMultiplier;
    public static double p,i,d,f,Target;
    private PIDController Controller;
    boolean intaked,outakeInited,outakeActionComplete;
    public static int INTIAL_OFFSET,PIXEL_LAYER,ALLOWED_ERROR;
    public double slidePower;
    /////////////////////////////////////////////
    @Override
    public void init(){
////////////////////////HARDWARE INIT////////////////
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        intake_arm = hardwareMap.get(ServoImplEx.class, "IA");
        claw = hardwareMap.get(ServoImplEx.class, "Claw");
        claw_angler = hardwareMap.get(ServoImplEx.class,"CA");

        aligner = hardwareMap.get(CRServo.class,"AL");

        claw_Position = hardwareMap.get(AnalogInput.class,"Claw Pos");
        claw_Angle = hardwareMap.get(AnalogInput.class,"Claw Angle");

        left_slides = hardwareMap.get(DcMotorEx.class,"LS");
        right_slides = hardwareMap.get(DcMotorEx.class,"RS");
        left_intake = hardwareMap.get(DcMotorEx.class,"LI");
        right_intake = hardwareMap.get(DcMotorEx.class,"RI");

////////////////////////SET PWM RANGE////////////////
        intake_arm.setPwmRange(new PwmControl.PwmRange(510,24900));
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
        p=0;i=0;d=0;f=0;Target = 0;speedMultiplier=1;
        INTIAL_OFFSET = 0;PIXEL_LAYER= 0;
        intaked = false; outakeInited = false;

    }
    @Override
    public void init_loop(){

    }
    @Override
    public void start(){

    }
    @Override
    public void loop(){
////////////////////////SLIDES PID//////////
        Controller.setPID(p,i,d);
        int Pos = (right_slides.getCurrentPosition()+left_slides.getCurrentPosition())/2;
        double PID = Controller.calculate(Pos,Target);
        double Power = PID+f;
        left_slides.setPower(Power);
        right_slides.setPower(Power);
//INTAKE
        if(gamepad1.right_trigger>0){
            left_intake.setPower(gamepad1.right_stick_x);
            right_intake.setPower(gamepad1.right_stick_x);
            aligner.setPower(1);
        }
        else if (gamepad1.b){
            left_intake.setPower(-1);
            right_intake.setPower(-1);
            aligner.setPower(1);
        }
        else {
            left_intake.setPower(0);
            right_intake.setPower(0);
            aligner.setPower(0);
        }
        if(gamepad1.dpad_up)intake_arm.setPosition(1);
        else if(gamepad1.dpad_down)intake_arm.setPosition(0);
//OUTTAKE
//>Slides
        if(gamepad1.a&&!outakeInited)outakeInited=true;
        if(outakeInited){
            if(Target==0) Target = INTIAL_OFFSET;
            else Target+=PIXEL_LAYER;
            if(getError(Pos,Target)<=ALLOWED_ERROR) outakeInited = false;
        }
//>Claw
        if(Target<INTIAL_OFFSET)claw_angler.setPosition(0);
        else if(Target>=INTIAL_OFFSET&&getError(Pos,Target)<=ALLOWED_ERROR)claw_angler.setPosition(1);
        if(gamepad1.left_bumper)claw_angler.setPosition(1);
        else if(gamepad1.right_bumper){
            claw_angler.setPosition(0);
            claw.setPosition(0.3);
        }
        else if(gamepad1.dpad_left) claw_angler.setPosition(0.3);
        if(gamepad1.x)claw.setPosition(1);
        else if(gamepad1.y)claw.setPosition(0);
////////////////////////DRIVE LOGIC//////////////////
        speedMultiplier = 1-gamepad1.left_trigger;
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y*speedMultiplier ,
                        -gamepad1.left_stick_x*speedMultiplier
                ),
                -gamepad1.right_stick_x*speedMultiplier
        ));

        drive.updatePoseEstimate();
        telemetry.addLine("Running: TechnoManiacs CenterStage Operating System - V0.1");

////////////////////////TELEMETRY////////////////////
        telemetry.addData("loop time: ",loopTime.time());
        telemetry.addData("Claw Position: ",clawPosition);
        telemetry.addData("Claw Angle: ",clawAngle);
        telemetry.addData("Slide Speed: ",Power);

        loopTime.reset();
        telemetry.update();
    }
    @Override
    public void stop(){

    }
    public static int getError(int current, double target){
        int error = Math.abs((int)target-current);
        return error;
    }
}
