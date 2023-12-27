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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;


@Config
@TeleOp
public class Driver_Mode extends OpMode
{
    /////////////////////////////////////////////
    ServoImplEx intake_arm,claw,claw_angler, drone;
    CRServo aligner;
    DcMotorEx left_slides,right_slides,left_intake,right_intake;
    AnalogInput claw_Position, claw_Angle;
    RevColorSensorV3 pixelDetector;
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    /////////////////////////////////////////////
    private ElapsedTime loopTime = new ElapsedTime();
    private MecanumDrive drive;
    public static double clawPosition,clawAngle,speedMultiplier;
    public static double zeroLimit,oneLimit,twoLimit;
    public static double p,i,d,f,Target;
    private PIDController Controller;
    boolean intaked,intakeReady, outtaked,actionInit;
    public static int INTAKE_OFFSET,INTIAL_OFFSET,PIXEL_LAYER,ALLOWED_ERROR;
    //700 is minimum
    public double slidePower;
    /////////////////////////////////////////////
    @Override
    public void init(){
////////////////////////HARDWARE INIT////////////////
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        intake_arm = hardwareMap.get(ServoImplEx.class, "IA");
        claw = hardwareMap.get(ServoImplEx.class, "Claw");
        claw_angler = hardwareMap.get(ServoImplEx.class,"CA");
        drone = hardwareMap.get(ServoImplEx.class, "Drone");

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
        drone.setPwmRange(new PwmControl.PwmRange(510,2490));
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
        p=0.0035;i=0;d=0.0001;f=0.075;Target = 0;speedMultiplier=1;
        INTIAL_OFFSET = 800;PIXEL_LAYER= 300;ALLOWED_ERROR=50;INTAKE_OFFSET=300;
        zeroLimit=80;oneLimit=66;twoLimit=55;
        intaked = false; intakeReady=false; outtaked = false; actionInit = false;
        claw_angler.setPosition(1);
        claw.setPosition(1);
        intake_arm.setPosition(1);
        pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
    }
    @Override
    public void init_loop(){

    }
    @Override
    public void start(){

    }
    @Override
    public void loop(){
        clawAngle=claw_Angle.getVoltage();
        clawPosition=claw_Position.getVoltage();
////////////////////LED LOGIC///////////
        /*if(pixelDetector.getDistance(DistanceUnit.MM)>twoLimit&&pixelDetector.getDistance(DistanceUnit.MM)<oneLimit){
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
        }*/
        if (gamepad2.b) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
        }
        else if (gamepad2.x) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        }
        else if (gamepad2.y) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
        }
        else if (gamepad2.a) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
        }
        else if (gamepad2.dpad_up) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        }
        else if (gamepad2.dpad_down) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        }
        blinkinLedDriver.setPattern(pattern);

        if(gamepad2.dpad_right){
            drone.setPosition(0);
        }

////////////////////////SLIDES PID//////////
        Controller.setPID(p, i, d);
        int Pos = (right_slides.getCurrentPosition() + left_slides.getCurrentPosition()) / 2;
        double PID = Controller.calculate(Pos, Target);
        double Power = PID + f;
            if(gamepad2.touchpad){
                left_slides.setPower(-0.1);
                right_slides.setPower(-0.1);
            }
            else {
                left_slides.setPower(Power);
                right_slides.setPower(Power);
            }
            if(gamepad1.options) Target=2200;
            if(gamepad1.touchpad) Target=1000;

//INTAKE
        if(gamepad1.right_trigger>0){
            left_intake.setPower(gamepad1.right_trigger);
            right_intake.setPower(gamepad1.right_trigger);
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
        else if(gamepad1.dpad_down)intake_arm.setPosition(0.16);
        if(gamepad1.a)actionInit=true;
        if(!intakeReady&&!intaked&&!outtaked&&actionInit){
            Target=INTAKE_OFFSET;
            claw.setPosition(0.5);
            if(getError(Pos,Target)<=ALLOWED_ERROR) {
                intakeReady = true;
                actionInit = false;
            }
        }
        else if(intakeReady&&!intaked&&!outtaked&&actionInit){
            Target=-50;
            if(getError(Pos,Target)<=ALLOWED_ERROR&&Pos>0){
                left_slides.setPower(-0.25);
                right_slides.setPower(-0.25);
            }
            if(Pos<10) {
                claw.setPosition(1);
                intaked=true;
                actionInit=false;
            }
        }
//OUTTAKE
//>Slides
        else if(intakeReady&&intaked&&!outtaked&&actionInit){
            if(Target==-50)Target=INTIAL_OFFSET;
            if(gamepad1.share)Target-=50;
            if(getError(Pos,Target)<=ALLOWED_ERROR){
                claw_angler.setPosition(0);
                if(gamepad1.right_bumper)Target+=PIXEL_LAYER;
                else if(gamepad1.left_bumper)Target-=PIXEL_LAYER;
                if(gamepad1.a) {
                    outtaked = true;

                }
            }
        }
        else if(intakeReady&&intaked&&outtaked){
            if(Target<800&&claw_angler.getPosition()!=1)Target=800;
            if(getError(Pos,Target)<=ALLOWED_ERROR){
                claw_angler.setPosition(1);
                claw.setPosition(0.5);
                if(clawAngle<1.5)Target=-50;
                if(Target==-50) {
                    intakeReady = false;
                    intaked = false;
                    outtaked = false;
                    actionInit = false;
                }
            }
        }
        if(gamepad1.start){
            //reset code
        }
//>Claw
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
        telemetry.addData("target:", Target);
        telemetry.addData("current:",Pos);

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
