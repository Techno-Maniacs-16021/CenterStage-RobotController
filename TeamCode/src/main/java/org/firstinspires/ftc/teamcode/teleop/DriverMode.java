package org.firstinspires.ftc.teamcode.teleop;

/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Config
@TeleOp
public class DriverMode extends LinearOpMode {

    // Declare OpMode members.
        ServoImplEx  left_tilt,right_tilt,swivel;
        CRServoImplEx left_arm, right_arm;
        CRServo left_intake, right_intake;
        DcMotor vertical_motor, horizontal_motor;
        AnalogInput rightArmPosition,leftArmPosition;

        private ElapsedTime runtime = new ElapsedTime();
        private ElapsedTime armCooldown = new ElapsedTime();
        private ElapsedTime wristCooldown = new ElapsedTime();
        private ElapsedTime verticalTrack = new ElapsedTime();
        private ElapsedTime horizontalTrack = new ElapsedTime();
        private ElapsedTime pidCooldown = new ElapsedTime();
        private ElapsedTime armTrack = new ElapsedTime();
        double armPosition=0;
        double drivePower = 1;
        double wristPosition=0.32;
        public static double vKp=0,vKi=0,vKd=0,vTarget=0,vIntSum=0,vLastError=0;
        public static double hKp=0,hKi=0,hKd=0,hTarget=0,hIntSum=0,hLastError=0;
        public static double kp=0.0032,kd=0.0001,ki=0,kf=0.21,targetAngle=-20,intSum=0,lastError=0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


            //Hardware declarations{
            left_arm = hardwareMap.get(CRServoImplEx.class,"LA");
            right_arm = hardwareMap.get(CRServoImplEx.class, "RA");
            left_tilt = hardwareMap.get(ServoImplEx.class,"LT");
            right_tilt = hardwareMap.get(ServoImplEx.class, "RT");
            swivel = hardwareMap.get(ServoImplEx.class, "swivel");
            left_intake = hardwareMap.get(CRServo.class,"LI");
            right_intake = hardwareMap.get(CRServo.class,"RI");
            vertical_motor = hardwareMap.get(DcMotor.class,"V");
            horizontal_motor = hardwareMap.get(DcMotor.class,"H");
            rightArmPosition = hardwareMap.get(AnalogInput.class,"rArmPos");
            leftArmPosition = hardwareMap.get(AnalogInput.class,"lArmPos");
            //}

            //Servo Pwm Range Declaration{
            left_arm.setPwmRange(new PwmControl.PwmRange(500,2500));
            right_arm.setPwmRange(new PwmControl.PwmRange(500,2500));
            left_tilt.setPwmRange(new PwmControl.PwmRange(510,2490));
            right_tilt.setPwmRange(new PwmControl.PwmRange(510,2490));
            swivel.setPwmRange(new PwmControl.PwmRange(510,2490));
            //}

            //Reversing Things{
            left_tilt.setDirection(ServoImplEx.Direction.REVERSE);
            right_arm.setDirection(CRServoImplEx.Direction.REVERSE);
            left_intake.setDirection(CRServo.Direction.REVERSE);
            vertical_motor.setDirection(DcMotorSimple.Direction.REVERSE);
            horizontal_motor.setDirection(DcMotorSimple.Direction.REVERSE);
            //}

            //Zero Power Behavior{
            horizontal_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            vertical_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //}
            //Servo Position Reset{
            left_tilt.setPosition(0.32);
            right_tilt.setPosition(0.32);
            swivel.setPosition(0);

            //}

            vertical_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            vertical_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            vertical_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            vertical_motor.setTargetPosition(0);




        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            double leftArmAngle = ((3.152 -leftArmPosition.getVoltage()) /3.3) * 360;
            double rightArmAngle = (rightArmPosition.getVoltage() /3.3) * 360;
            double armAngle = (leftArmAngle+rightArmAngle)/2;

            if(targetAngle==-20&&armAngle<90){
                left_arm.setPower(0);
                right_arm.setPower(0);
            }
            else if(Math.abs(targetAngle-armAngle)>3){
                double pos = armAngle;
                double error = targetAngle-pos;
                double derivative = (error-lastError)/armTrack.seconds();
                intSum = intSum + (error*armTrack.seconds());
                if (intSum > 100) {
                    intSum = 100;
                }
                if (intSum < -100) {
                    intSum = -100;
                }
                double out = (kf*Math.cos(Math.toRadians(armAngle-58)))+(kp*error)+(ki*intSum)+(kd*derivative);
                left_arm.setPower(out);
                right_arm.setPower(out);
                lastError=error;
                armTrack.reset();
            }
            if(gamepad1.left_trigger>0)targetAngle = -20;
            else if(gamepad1.y&&gamepad1.right_trigger>0)targetAngle=80;
            else if(gamepad1.right_trigger>0)targetAngle=160;
            if(gamepad2.dpad_up){
                vertical_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                vertical_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if(Math.abs(vTarget-vertical_motor.getCurrentPosition())>5){
                double pos = vertical_motor.getCurrentPosition();
                double error = vTarget-pos;
                double derivative = (error-vLastError)/verticalTrack.seconds();
                vIntSum = vIntSum + (error*verticalTrack.seconds());
                double out = (vKp*error)+(vKi*vIntSum)+(vKd*derivative);
                vertical_motor.setPower(out);
                vLastError=error;
                verticalTrack.reset();
            }
            if(gamepad1.left_stick_y>0) vTarget = 5000;
            else if(gamepad1.left_stick_y<0) vTarget = 0;
            if(gamepad1.left_stick_x>0) hTarget = 1000;
            else if(gamepad1.left_stick_x<0) hTarget = 0;
            if(gamepad1.right_trigger>0&&armCooldown.time()>=.075){
                //armPosition+=0.05*gamepad1.right_trigger;
                armPosition=0.1;
                left_arm.setPwmEnable();
                right_arm.setPwmEnable();
                armCooldown.reset();
            }
            else if(gamepad1.left_trigger>0&&armCooldown.time()>=.075){
                //armPosition-=0.05*gamepad1.left_trigger;
                armPosition=0;
                left_arm.setPwmDisable();
                right_arm.setPwmDisable();
                armCooldown.reset();
            }
                if(gamepad1.dpad_left&&wristCooldown.time()>0.01) {
                wristPosition += 0.01;
                wristCooldown.reset();
            }
            else if(armAngle>100){
                wristPosition=0.39;
            }
            else if(gamepad1.right_bumper){
                wristPosition=0.01;
            }
            else if(gamepad1.left_bumper){
                wristPosition=0.32;
            }
            if(targetAngle<100)swivel.setPosition(0);
            else if(armAngle>100)swivel.setPosition(.6);
            vertical_motor.setTargetPosition((int)vTarget);
            horizontal_motor.setPower(gamepad1.left_stick_x);
            left_tilt.setPosition(wristPosition);
            right_tilt.setPosition(wristPosition);
            if(gamepad1.a) {
                left_intake.setPower(1);
                right_intake.setPower(1);
            }
            else if(gamepad1.b){
                left_intake.setPower(-1);
                right_intake.setPower(-1);
            }
            else {
                left_intake.setPower(0);
                right_intake.setPower(0);
            }
            if(gamepad2.left_trigger>0.5 && gamepad2.right_trigger<0.5){

                drivePower=0.8;
            }else if(gamepad2.right_trigger>0.5 && gamepad2.left_trigger <0.5){
                drivePower=0.4;
            }else if(gamepad2.right_trigger>0.5 && gamepad2.left_trigger >0.5){
                drivePower=0.2;
            }else{
                drivePower=1;
            }
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            drive.updatePoseEstimate();

            PoseVelocity2d poseEstimate = drive.updatePoseEstimate();
            telemetry.addData("wristPosition: ",wristPosition);
            telemetry.addData("power multiplier: ",drivePower);
            telemetry.addLine("Running: TechnoManiacs Operating System - V0.5");

            telemetry.update();
        }
    }
}
