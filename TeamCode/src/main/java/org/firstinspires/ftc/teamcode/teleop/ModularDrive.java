package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bot.BotV1;

public class ModularDrive extends OpMode {
    private BotV1 bot;
    private ElapsedTime loopTime;
    @Override
    public void init() {
        bot = new BotV1(hardwareMap, new Pose2d(0,0,0));
        loopTime = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        updateBotState();
        processControllerOneInputs();
        processControllerTwoInputs();
        updateTelemetry();
    }

    private void processControllerOneInputs(){
        if(gamepad1.options) bot.setTarget(2200);
        if(gamepad1.touchpad) bot.setTarget(1000);
        if(gamepad1.right_trigger > 0) bot.intake(gamepad1.right_trigger);
        else if(gamepad2.b) bot.intake(-1);
        else bot.intake(0);
        if(gamepad1.dpad_up) bot.raiseIntake();
        else if(gamepad1.dpad_down) bot.lowerIntake();
        if(gamepad1.a) bot.doCustomSequence(true);
        bot.sequence(false);
        if(gamepad1.right_bumper && bot.isDoSequence()) bot.addTarget(bot.getPIXEL_LAYER());
        else if(gamepad1.left_bumper && bot.isDoSequence()) bot.addTarget(-bot.getPIXEL_LAYER());
        if(gamepad1.start){
            //reset code
        }
        if(gamepad1.x) bot.closeClaw();
        else if(gamepad1.y) bot.openClaw();
        else if(gamepad1.dpad_right) bot.activateClawAngler();

        double speedMultiplier = 1-gamepad1.left_trigger;
        bot.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y*speedMultiplier ,
                        -gamepad1.left_stick_x*speedMultiplier
                ),
                -gamepad1.right_stick_x*speedMultiplier
        ));
    }

    private void processControllerTwoInputs(){
        if (gamepad2.b) bot.setLightColor("yellow");
        else if (gamepad2.x) bot.setLightColor("green");
        else if (gamepad2.y) bot.setLightColor("violet");
        else if (gamepad2.a) bot.setLightColor("white");
        else if (gamepad2.dpad_up) bot.setLightColor("blue");
        else if (gamepad2.dpad_down) bot.setLightColor("red");
        if(gamepad2.dpad_right) bot.shootDrone();
        if(gamepad2.touchpad) bot.slowlyLowerSlides();
        else bot.slidesGoToTarget();
    }

    private void updateTelemetry(){
        telemetry.addLine("Running: TechnoManiacs CenterStage Operating System - V0.1");
        telemetry.addData("loop time: ", loopTime.time());
        telemetry.addData("Claw Position: ", bot.getClawPosition());
        telemetry.addData("Claw Angle: ", bot.getClawAngle());
        telemetry.addData("Slide Speed: ", bot.getPower());
        telemetry.addData("target:", bot.getSlidesTarget());
        telemetry.addData("current:", bot.getSlidesPosition());
        loopTime.reset();
        telemetry.update();
    }

    private void updateBotState(){
        bot.updateServos();
        bot.updatePoseEstimate();
        bot.updateSlidePosition();
    }
}
