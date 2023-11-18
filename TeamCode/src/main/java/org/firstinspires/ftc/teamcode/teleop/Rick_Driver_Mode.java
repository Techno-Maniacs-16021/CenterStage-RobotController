package org.firstinspires.ftc.teamcode.teleop;

import static android.os.SystemClock.sleep;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import android.widget.HorizontalScrollView;

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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;


@Config
@TeleOp
public class Rick_Driver_Mode extends OpMode
{
    /////////////////////////////////////////////
    ServoImplEx intake_arm,claw,claw_angler;
    CRServo left_aligner, right_aligner;
    DcMotorEx left_slides,right_slides,left_intake,right_intake;
    AnalogInput claw_Position, claw_Angle;
    /////////////////////////////////////////////
    private ElapsedTime loopTime = new ElapsedTime();
    private MecanumDrive drive;
    public static double clawPosition,clawAngle,intakePower,speedMultiplier;
    boolean intaked,outaked,OVERIDE;
    public static int HIGH,MID;
    public double slidePower;
    /////////////////////////////////////////////
    @Override
    public void init(){
////////////////////////HARDWARE INIT////////////////
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        intake_arm = hardwareMap.get(ServoImplEx.class, "IA");
        claw = hardwareMap.get(ServoImplEx.class, "Claw");
        claw_angler = hardwareMap.get(ServoImplEx.class,"CA");

        left_aligner = hardwareMap.get(CRServo.class,"LA");
        right_aligner = hardwareMap.get(CRServo.class,"RA");

        claw_Position = hardwareMap.get(AnalogInput.class,"Claw Pos");
        claw_Angle = hardwareMap.get(AnalogInput.class,"Claw Angle");

        left_slides = hardwareMap.get(DcMotorEx.class,"LS");
        right_slides = hardwareMap.get(DcMotorEx.class,"RS");
        left_intake = hardwareMap.get(DcMotorEx.class,"LI");
        right_intake = hardwareMap.get(DcMotorEx.class,"RI");

////////////////////////SET PWM RANGE////////////////
        intake_arm.setPwmRange(new PwmControl.PwmRange(500,2500));
        claw.setPwmRange(new PwmControl.PwmRange(500,2500));
        claw_angler.setPwmRange(new PwmControl.PwmRange(500,2500));
////////////////////////HARDWARE REVERSING///////////
left_slides.setDirection(DcMotorSimple.Direction.REVERSE);
right_intake.setDirection(DcMotorSimple.Direction.REVERSE);
////////////////////////MOTOR BRAKE BEHAVIOR/////////
        left_slides.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right_slides.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
////////////////////////ENCODER RESET////////////////
        left_slides.setMode(RUN_WITHOUT_ENCODER);
        right_slides.setMode(RUN_WITHOUT_ENCODER);
////////////////////////DASHBOARD TELEMETRY//////////
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
////////////////////////STATUS UPDATE////////////////
        telemetry.addData("Status", "Initialized");
/////////////////////////////////////////////////////
        slidePower=0;

    }
    @Override
    public void init_loop(){

    }
    @Override
    public void start(){

    }
    @Override
    public void loop(){
//INTAKE
        if(gamepad1.a){
            left_intake.setPower(0.5);
            right_intake.setPower(0.5);
            left_aligner.setPower(1);
            right_aligner.setPower(1);
        }
        else if (gamepad1.b){
            left_intake.setPower(-1);
            right_intake.setPower(-1);
            left_aligner.setPower(0);
            right_aligner.setPower(0);
        }
        else {
            left_intake.setPower(0);
            right_intake.setPower(0);
            left_aligner.setPower(0);
            right_aligner.setPower(0);
        }
        if(gamepad1.dpad_up)intake_arm.setPosition(1);
        else if(gamepad1.dpad_down)intake_arm.setPosition(0);
//Slides
        left_slides.setPower(slidePower);
        right_slides.setPower(slidePower);
        if(gamepad1.right_trigger!=0) slidePower = gamepad1.right_trigger;
        else if(gamepad1.left_trigger!=0) slidePower = -gamepad1.left_trigger;
        else slidePower = 0;
//Outtake
        if(gamepad1.left_bumper)claw_angler.setPosition(1);
        else if(gamepad1.right_bumper){
            claw_angler.setPosition(0);
            claw.setPosition(0.3);
        }
        if(gamepad1.x)claw.setPosition(1);
        else if(gamepad1.y)claw.setPosition(0);
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
        telemetry.addData("left slide pos:",left_slides.getCurrentPosition());
        telemetry.addData("claw position: ",clawPosition);
        telemetry.addData("claw angle: ",claw);
        loopTime.reset();
        telemetry.update();
    }
    @Override
    public void stop(){

    }

}
