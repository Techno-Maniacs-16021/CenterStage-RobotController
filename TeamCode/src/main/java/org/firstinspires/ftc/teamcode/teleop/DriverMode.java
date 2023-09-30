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

    // Config

        public double clawTarget = 1;
        public double armTarget = 1;

    // Declare OpMode members.
        CRServo left_aligner, right_aligner;
        DcMotor linear1, linear2, intake1, intake2;
        AnalogInput claw, intake_arm;

        /*
        Driver 1 tasks:
        - driving (joysticks) - hold
        - motor intake (a,b) - hold
        - intake arm (rt trig) - switch
        Driver 2 tasks:
        - claw (b) - switch
        - linear slides (dpad up/down) - hold
        - aligners (a) - switch
         */
        private ElapsedTime runtime = new ElapsedTime();

        double armPosition = 0;
        double clawPosition = 0;
        double drivePower = 1;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


            //Hardware declarations{
            left_aligner = hardwareMap.get(CRServo.class,"LA");
            right_aligner = hardwareMap.get(CRServo.class,"RA");
            intake1 = hardwareMap.get(DcMotor.class,"I1");
            intake2 = hardwareMap.get(DcMotor.class,"I2");
            linear1 = hardwareMap.get(DcMotor.class,"L1");
            linear2 = hardwareMap.get(DcMotor.class,"L2");
            claw = hardwareMap.get(AnalogInput.class,"claw");
            intake_arm = hardwareMap.get(AnalogInput.class,"intake");
            //}

            //Servo Pwm Range Declaration{
        //.setPwmRange(new PwmControl.PwmRange(500,2500));
            //}

            //Reversing Things{
            //.setDirection(DcMotorSimple.Direction.REVERSE);
            //}

            //Zero Power Behavior{
            linear1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            linear2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //}
            //Servo Position Reset{
            //.setPosition(0);

            //}

            linear1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linear2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            linear1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            linear2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            linear1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linear2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            linear1.setTargetPosition(0);
            linear2.setTargetPosition(0);




        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            //linear slides (D2) - hold

            if(gamepad2.dpad_up){
                linear2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                linear1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                linear2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linear1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            //claw (D2) - switch

            if(gamepad2.b && clawPosition == 0) {
                clawPosition = clawTarget;
                //claw.setPosition(clawPosition);
            }
            else if (gamepad2.b && clawPosition == clawTarget) {
                clawPosition = 0;
                //claw.setPosition(clawPosition);
            }

            //intake arm (D1) - switch

            if(gamepad1.right_trigger > 0 && armPosition == 0) {
                armPosition = armTarget;
                //intake_arm.setPosition(armPosition);
            }
            else if(gamepad1.right_trigger > 0 && armPosition == armTarget) {
                armPosition = 0;
                //intake_arm.setPosition(armPosition);
            }

            //aligners (D2) - switch

            if(gamepad2.a && left_aligner.getPower() == 0) {
                left_aligner.setPower(1);
                right_aligner.setPower(1);
            }
            else if (gamepad2.a && left_aligner.getPower() == 1) {
                left_aligner.setPower(0);
                right_aligner.setPower(0);
            }

            //intake motors (D1) - hold

            if (gamepad1.a) {
                intake1.setPower(1);
                intake2.setPower(1);
            }
            else if (gamepad1.b) {
                intake1.setPower(-1);
                intake2.setPower(-1);
            }
            else {
                intake1.setPower(0);
                intake2.setPower(0);
            }

            //drive power

            if(gamepad2.left_trigger>0.5 && gamepad2.right_trigger<0.5){

                drivePower=0.8;
            }else if(gamepad2.right_trigger>0.5 && gamepad2.left_trigger <0.5){
                drivePower=0.4;
            }else if(gamepad2.right_trigger>0.5 && gamepad2.left_trigger >0.5){
                drivePower=0.2;
            }else{
                drivePower=1;
            }

            //driving (D1) - hold

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y * drivePower,
                            -gamepad1.left_stick_x * drivePower
                    ),
                    -gamepad1.right_stick_x * drivePower
            ));

            drive.updatePoseEstimate();

            PoseVelocity2d poseEstimate = drive.updatePoseEstimate();
            telemetry.addData("power multiplier: ",drivePower);
            telemetry.addLine("Running: TechnoManiacs CenterStage Operating System - V0.1");

            telemetry.update();
        }
    }
}
