package org.firstinspires.ftc.teamcode.bot;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class AutoBot {
    ServoImplEx intake_arm,claw,claw_angler;
    CRServo aligner;
    DcMotorEx left_slides,right_slides,left_intake,right_intake;
    AnalogInput claw_Position, claw_Angle;
    RevColorSensorV3 pixelDetector;
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    public AutoBot(ServoImplEx intake_arm, ServoImplEx claw, ServoImplEx claw_angler, CRServo aligner, DcMotorEx left_slides, DcMotorEx right_slides, DcMotorEx left_intake, DcMotorEx right_intake, AnalogInput claw_Position, AnalogInput claw_Angle, RevColorSensorV3 pixelDetector, RevBlinkinLedDriver blinkinLedDriver, RevBlinkinLedDriver.BlinkinPattern pattern) {
        this.intake_arm = intake_arm;
        this.claw = claw;
        this.claw_angler = claw_angler;
        this.aligner = aligner;
        this.left_slides = left_slides;
        this.right_slides = right_slides;
        this.left_intake = left_intake;
        this.right_intake = right_intake;
        this.claw_Position = claw_Position;
        this.claw_Angle = claw_Angle;
        this.pixelDetector = pixelDetector;
        this.blinkinLedDriver = blinkinLedDriver;
        this.pattern = pattern;
    }


}
