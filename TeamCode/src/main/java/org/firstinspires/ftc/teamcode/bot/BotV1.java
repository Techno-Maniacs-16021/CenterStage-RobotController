package org.firstinspires.ftc.teamcode.bot;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class BotV1 extends MecanumDrive {
    private ServoImplEx intakeArmServo, clawServo, clawAnglerServo, droneServo;
    private CRServo alignerCRServo;
    private DcMotorEx leftSlidesMotor, rightSlidesMotor, leftIntakeMotor, rightIntakeMotor;
    private AnalogInput clawPositionAnalogInput, clawAngleAnalogInput;
    private RevColorSensorV3 pixelColorSensor;
    private RevBlinkinLedDriver blinkinLedDriver;
    private double clawPosition, clawAngle, speedMultiplier;
    private double zeroLimit, oneLimit, twoLimit;
    private double p, i, d, f, slidesTarget, slidesPosition;

    public ServoImplEx getIntakeArmServo() {
        return intakeArmServo;
    }

    public void setIntakeArmServo(ServoImplEx intakeArmServo) {
        this.intakeArmServo = intakeArmServo;
    }

    public ServoImplEx getClawServo() {
        return clawServo;
    }

    public void setClawServo(ServoImplEx clawServo) {
        this.clawServo = clawServo;
    }

    public ServoImplEx getClawAnglerServo() {
        return clawAnglerServo;
    }

    public void setClawAnglerServo(ServoImplEx clawAnglerServo) {
        this.clawAnglerServo = clawAnglerServo;
    }

    public ServoImplEx getDroneServo() {
        return droneServo;
    }

    public void setDroneServo(ServoImplEx droneServo) {
        this.droneServo = droneServo;
    }

    public CRServo getAlignerCRServo() {
        return alignerCRServo;
    }

    public void setAlignerCRServo(CRServo alignerCRServo) {
        this.alignerCRServo = alignerCRServo;
    }

    public DcMotorEx getLeftSlidesMotor() {
        return leftSlidesMotor;
    }

    public void setLeftSlidesMotor(DcMotorEx leftSlidesMotor) {
        this.leftSlidesMotor = leftSlidesMotor;
    }

    public DcMotorEx getRightSlidesMotor() {
        return rightSlidesMotor;
    }

    public void setRightSlidesMotor(DcMotorEx rightSlidesMotor) {
        this.rightSlidesMotor = rightSlidesMotor;
    }

    public DcMotorEx getLeftIntakeMotor() {
        return leftIntakeMotor;
    }

    public void setLeftIntakeMotor(DcMotorEx leftIntakeMotor) {
        this.leftIntakeMotor = leftIntakeMotor;
    }

    public DcMotorEx getRightIntakeMotor() {
        return rightIntakeMotor;
    }

    public void setRightIntakeMotor(DcMotorEx rightIntakeMotor) {
        this.rightIntakeMotor = rightIntakeMotor;
    }

    public AnalogInput getClawPositionAnalogInput() {
        return clawPositionAnalogInput;
    }

    public void setClawPositionAnalogInput(AnalogInput clawPositionAnalogInput) {
        this.clawPositionAnalogInput = clawPositionAnalogInput;
    }

    public AnalogInput getClawAngleAnalogInput() {
        return clawAngleAnalogInput;
    }

    public void setClawAngleAnalogInput(AnalogInput clawAngleAnalogInput) {
        this.clawAngleAnalogInput = clawAngleAnalogInput;
    }

    public RevColorSensorV3 getPixelColorSensor() {
        return pixelColorSensor;
    }

    public void setPixelColorSensor(RevColorSensorV3 pixelColorSensor) {
        this.pixelColorSensor = pixelColorSensor;
    }

    public RevBlinkinLedDriver getBlinkinLedDriver() {
        return blinkinLedDriver;
    }

    public void setBlinkinLedDriver(RevBlinkinLedDriver blinkinLedDriver) {
        this.blinkinLedDriver = blinkinLedDriver;
    }

    public double getClawPosition() {
        return clawPosition;
    }

    public void setClawPosition(double clawPosition) {
        this.clawPosition = clawPosition;
    }

    public double getClawAngle() {
        return clawAngle;
    }

    public void setClawAngle(double clawAngle) {
        this.clawAngle = clawAngle;
    }

    public double getSpeedMultiplier() {
        return speedMultiplier;
    }

    public void setSpeedMultiplier(double speedMultiplier) {
        this.speedMultiplier = speedMultiplier;
    }

    public double getZeroLimit() {
        return zeroLimit;
    }

    public void setZeroLimit(double zeroLimit) {
        this.zeroLimit = zeroLimit;
    }

    public double getOneLimit() {
        return oneLimit;
    }

    public void setOneLimit(double oneLimit) {
        this.oneLimit = oneLimit;
    }

    public double getTwoLimit() {
        return twoLimit;
    }

    public void setTwoLimit(double twoLimit) {
        this.twoLimit = twoLimit;
    }

    public double getP() {
        return p;
    }

    public void setP(double p) {
        this.p = p;
    }

    public double getI() {
        return i;
    }

    public void setI(double i) {
        this.i = i;
    }

    public double getD() {
        return d;
    }

    public void setD(double d) {
        this.d = d;
    }

    public double getF() {
        return f;
    }

    public void setF(double f) {
        this.f = f;
    }

    public double getSlidesTarget() {
        return slidesTarget;
    }

    public void setSlidesTarget(double slidesTarget) {
        this.slidesTarget = slidesTarget;
    }

    public double getSlidesPosition() {
        return slidesPosition;
    }

    public void setSlidesPosition(double slidesPosition) {
        this.slidesPosition = slidesPosition;
    }

    public PIDController getController() {
        return Controller;
    }

    public void setController(PIDController controller) {
        Controller = controller;
    }

    public boolean isDoSequence() {
        return doSequence;
    }

    public void setDoSequence(boolean doSequence) {
        this.doSequence = doSequence;
    }

    public boolean isThirdSequence() {
        return thirdSequence;
    }

    public void setThirdSequence(boolean thirdSequence) {
        this.thirdSequence = thirdSequence;
    }

    public int getINTAKE_OFFSET() {
        return INTAKE_OFFSET;
    }

    public void setINTAKE_OFFSET(int INTAKE_OFFSET) {
        this.INTAKE_OFFSET = INTAKE_OFFSET;
    }

    public int getINTIAL_OFFSET() {
        return INTIAL_OFFSET;
    }

    public void setINTIAL_OFFSET(int INTIAL_OFFSET) {
        this.INTIAL_OFFSET = INTIAL_OFFSET;
    }

    public int getPIXEL_LAYER() {
        return PIXEL_LAYER;
    }

    public void setPIXEL_LAYER(int PIXEL_LAYER) {
        this.PIXEL_LAYER = PIXEL_LAYER;
    }

    public int getALLOWED_ERROR() {
        return ALLOWED_ERROR;
    }

    public void setALLOWED_ERROR(int ALLOWED_ERROR) {
        this.ALLOWED_ERROR = ALLOWED_ERROR;
    }

    public int getCurrentSequence() {
        return currentSequence;
    }

    public void setCurrentSequence(int currentSequence) {
        this.currentSequence = currentSequence;
    }

    private PIDController Controller;
    private boolean doSequence, thirdSequence;
    private int INTAKE_OFFSET, INTIAL_OFFSET, PIXEL_LAYER, ALLOWED_ERROR;

    private int currentSequence;

    public BotV1(HardwareMap hardwareMap, Pose2d pose) {

        ////////////////////////SET MecanumDrive////////////////

        super(hardwareMap, pose);

        ////////////////////////SET Hardware////////////////

        intakeArmServo = hardwareMap.get(ServoImplEx.class, "IA");
        clawServo = hardwareMap.get(ServoImplEx.class, "Claw");
        clawAnglerServo = hardwareMap.get(ServoImplEx.class,"CA");
        droneServo = hardwareMap.get(ServoImplEx.class, "Drone");

        alignerCRServo = hardwareMap.get(CRServo.class,"Aligner");

        clawPositionAnalogInput = hardwareMap.get(AnalogInput.class,"Claw Pos");
        clawAngleAnalogInput = hardwareMap.get(AnalogInput.class,"Claw Angle");

        leftSlidesMotor = hardwareMap.get(DcMotorEx.class,"LS");
        rightSlidesMotor = hardwareMap.get(DcMotorEx.class,"RS");
        leftIntakeMotor = hardwareMap.get(DcMotorEx.class,"LI");
        rightIntakeMotor = hardwareMap.get(DcMotorEx.class,"RI");

        pixelColorSensor = hardwareMap.get(RevColorSensorV3.class,"PD");

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "led");

        ////////////////////////SET PWM RANGE////////////////

        droneServo.setPwmRange(new PwmControl.PwmRange(510,2490));
        intakeArmServo.setPwmRange(new PwmControl.PwmRange(510,2490));
        clawServo.setPwmRange(new PwmControl.PwmRange(510,2490));
        clawAnglerServo.setPwmRange(new PwmControl.PwmRange(510,2490));

        ////////////////////////HARDWARE REVERSING///////////

        leftSlidesMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        ////////////////////////MOTOR BRAKE BEHAVIOR/////////

        leftSlidesMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightSlidesMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        ////////////////////////ENCODER RESET////////////////

        leftSlidesMotor.setMode(RUN_WITHOUT_ENCODER);
        rightSlidesMotor.setMode(RUN_WITHOUT_ENCODER);
        leftIntakeMotor.setMode(STOP_AND_RESET_ENCODER);
        leftIntakeMotor.setMode(RUN_WITHOUT_ENCODER);

        ////////////////////////PID CONTROLLERS//////////////

        Controller = new PIDController(p,i,d);

        ////////////////////////MATH//////////////

        p=0.0035;i=0;d=0.0001;f=0.075;
        slidesTarget = 0;speedMultiplier=1;
        INTIAL_OFFSET = 800;PIXEL_LAYER= 300;ALLOWED_ERROR=50;INTAKE_OFFSET=300;
        zeroLimit=80;oneLimit=66;twoLimit=55;
        clawAnglerServo.setPosition(1);
        clawServo.setPosition(1);
        intakeArmServo.setPosition(1);
        currentSequence = 0;
    }

    public void setLightColor(String color){
        switch (color.toLowerCase()){
            case "rainbow rainbow palette":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
                break;
            case "rainbow party palette":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);
                break;
            case "rainbow ocean palette":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);
                break;
            case "rainbow lava palette":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE);
                break;
            case "rainbow forest palette":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_FOREST_PALETTE);
                break;
            case "rainbow with glitter":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER);
                break;
            case "confetti":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
                break;
            case "shot red":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_RED);
                break;
            case "shot blue":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_BLUE);
                break;
            case "shot white":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE);
                break;
            case "sinelon rainbow palette":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_RAINBOW_PALETTE);
                break;
            case "sinelon party palette":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_PARTY_PALETTE);
                break;
            case "sinelon ocean palette":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_OCEAN_PALETTE);
                break;
            case "sinelon lava palette":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_LAVA_PALETTE);
                break;
            case "sinelon forest palette":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_FOREST_PALETTE);
                break;
            case "beats per minute rainbow palette":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE);
                break;
            case "beats per minute party palette":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_PARTY_PALETTE);
                break;
            case "beats per minute ocean palette":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_OCEAN_PALETTE);
                break;
            case "beats per minute lava palette":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_LAVA_PALETTE);
                break;
            case "beats per minute forest palette":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_FOREST_PALETTE);
                break;
            case "fire medium":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.FIRE_MEDIUM);
                break;
            case "fire large":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE);
                break;
            case "twinkles rainbow palette":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_RAINBOW_PALETTE);
                break;
            case "twinkles party palette":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_PARTY_PALETTE);
                break;
            case "twinkles ocean palette":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_OCEAN_PALETTE);
                break;
            case "twinkles lava palette":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_LAVA_PALETTE);
                break;
            case "twinkles forest palette":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_FOREST_PALETTE);
                break;
            case "color waves rainbow palette":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE);
                break;
            case "color waves party palette":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_PARTY_PALETTE);
                break;
            case "color waves ocean palette":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
                break;
            case "color waves lava palette":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE);
                break;
            case "color waves forest palette":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);
                break;
            case "larson scanner red":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_RED);
                break;
            case "larson scanner gray":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_GRAY);
                break;
            case "light chase red":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED);
                break;
            case "light chase blue":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE);
                break;
            case "light chase gray":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_GRAY);
                break;
            case "heartbeat red":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
                break;
            case "heartbeat blue":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
                break;
            case "heartbeat white":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE);
                break;
            case "heartbeat gray":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_GRAY);
                break;
            case "breath red":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);
                break;
            case "breath blue":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);
                break;
            case "breath gray":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_GRAY);
                break;
            case "strobe red":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
                break;
            case "strobe blue":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
                break;
            case "strobe gold":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
                break;
            case "strobe white":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);
                break;
            case "cp1 end to end blend to black":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_END_TO_END_BLEND_TO_BLACK);
                break;
            case "cp1 larson scanner":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_LARSON_SCANNER);
                break;
            case "cp1 light chase":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_LIGHT_CHASE);
                break;
            case "cp1 heartbeat slow":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_SLOW);
                break;
            case "cp1 heartbeat medium":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_MEDIUM);
                break;
            case "cp1 heartbeat fast":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_FAST);
                break;
            case "cp1 breath slow":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_BREATH_SLOW);
                break;
            case "cp1 breath fast":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_BREATH_FAST);
                break;
            case "cp1 shot":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_SHOT);
                break;
            case "cp1 strobe":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_STROBE);
                break;
            case "cp2 end to end blend to black":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_END_TO_END_BLEND_TO_BLACK);
                break;
            case "cp2 larson scanner":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_LARSON_SCANNER);
                break;
            case "cp2 light chase":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_LIGHT_CHASE);
                break;
            case "cp2 heartbeat slow":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_SLOW);
                break;
            case "cp2 heartbeat medium":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_MEDIUM);
                break;
            case "cp2 heartbeat fast":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_FAST);
                break;
            case "cp2 breath slow":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_BREATH_SLOW);
                break;
            case "cp2 breath fast":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_BREATH_FAST);
                break;
            case "cp2 shot":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_SHOT);
                break;
            case "cp2 strobe":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_STROBE);
                break;
            case "cp1 2 sparkle 1 on 2":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_SPARKLE_1_ON_2);
                break;
            case "cp1 2 sparkle 2 on 1":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_SPARKLE_2_ON_1);
                break;
            case "cp1 2 color gradient":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_GRADIENT);
                break;
            case "cp1 2 beats per minute":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_BEATS_PER_MINUTE);
                break;
            case "cp1 2 end to end blend 1 to 2":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_END_TO_END_BLEND_1_TO_2);
                break;
            case "cp1 2 end to end blend":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_END_TO_END_BLEND);
                break;
            case "cp1 2 no blending":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_NO_BLENDING);
                break;
            case "cp1 2 twinkles":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_TWINKLES);
                break;
            case "cp1 2 color waves":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_WAVES);
                break;
            case "cp1 2 sinelon":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_SINELON);
                break;
            case "hot pink":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                break;
            case "dark red":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_RED);
                break;
            case "red":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                break;
            case "red orange":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);
                break;
            case "orange":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
                break;
            case "gold":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
                break;
            case "yellow":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                break;
            case "lawn green":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LAWN_GREEN);
                break;
            case "lime":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIME);
                break;
            case "dark green":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);
                break;
            case "green":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                break;
            case "blue green":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN);
                break;
            case "aqua":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA);
                break;
            case "sky blue":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.SKY_BLUE);
                break;
            case "dark blue":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);
                break;
            case "blue":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                break;
            case "blue violet":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
                break;
            case "violet":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                break;
            case "white":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
                break;
            case "gray":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GRAY);
                break;
            case "dark gray":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GRAY);
                break;
            case "black":
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                break;

        }
    }
    public void updateServos(){
        clawAngle = clawAngleAnalogInput.getVoltage();
        clawPosition = clawPositionAnalogInput.getVoltage();
    }
    public void shootDrone(){
        droneServo.setPosition(0);
    }
    public void slidesGoToTarget(){
        Controller.setPID(p, i, d);
        slidesPosition = (rightSlidesMotor.getCurrentPosition() + leftSlidesMotor.getCurrentPosition()) / 2.0;
        double PID = Controller.calculate(slidesPosition, slidesTarget);
        double power = PID + f;
        leftSlidesMotor.setPower(power);
        rightSlidesMotor.setPower(power);
    }
    public void slowlyLowerSlides(){
        leftSlidesMotor.setPower(-0.1);
        rightSlidesMotor.setPower(-0.1);
        slidesPosition = (rightSlidesMotor.getCurrentPosition() + leftSlidesMotor.getCurrentPosition()) / 2.0;
    }

    public void setSlidesPower(double power){
        leftSlidesMotor.setPower(power);
        rightSlidesMotor.setPower(power);
        slidesPosition = (rightSlidesMotor.getCurrentPosition() + leftSlidesMotor.getCurrentPosition()) / 2.0;
    }
    public void setTarget(int target){
        this.slidesTarget = target;
    }

    public void addTarget(int target){
        if(slidesReachedTarget())
            this.slidesTarget += target;
    }

    public void intake(double power){
        leftIntakeMotor.setPower(power);
        rightIntakeMotor.setPower(power);
        alignerCRServo.setPower(power == 0 ? 0 : 1);
    }

    public void raiseIntake(){
        intakeArmServo.setPosition(1);
    }

    public void lowerIntake(){
        intakeArmServo.setPosition(0.16);
    }

    public void sequence(boolean aState){
        if(doSequence){
            switch(currentSequence) {
                case 0:
                    slidesTarget = INTAKE_OFFSET;
                    if(slidesReachedTarget()) {
                        currentSequence++;
                        doSequence = false;
                    }
                    break;
                case 1:
                    slidesTarget = -50;
                    if(slidesReachedTarget() && slidesPosition > 0)
                        setSlidesPower(-0.25);
                    if(slidesPosition<10) {
                        clawServo.setPosition(1);
                        currentSequence++;
                        doSequence = false;
                    }
                    break;
                case 2:
                    if(slidesReachedTarget()){
                        clawAnglerServo.setPosition(0);
//                        if(gamepad1.right_bumper)Target+=PIXEL_LAYER;
//                        else if(gamepad1.left_bumper)Target-=PIXEL_LAYER;
                        if(aState) {
                            currentSequence++;
                            doSequence = false;
                        }
                    }
                    break;
                case 3:
                    if(slidesTarget<800 && clawAnglerServo.getPosition()!=1) slidesTarget=800;
                    if(slidesReachedTarget()){
                        clawAnglerServo.setPosition(1);
                        clawServo.setPosition(0.5);
                        if(clawAngle < 1.5) slidesTarget=-50;
                        if(slidesTarget == -50) {
                            currentSequence = 0;
                            doSequence = false;
                        }
                    }
                    break;
            }
        }
    }
    public void doCustomSequence(boolean aState){
        doSequence = true;
        sequence(aState);
        if(currentSequence == 2 && !thirdSequence){
            slidesTarget = INTIAL_OFFSET;
            thirdSequence = true;
        }
    }
    public boolean slidesReachedTarget(){
        return getError((int)slidesPosition, slidesTarget) <= ALLOWED_ERROR;
    }
    public static int getError(int current, double target){
        int error = Math.abs((int)target-current);
        return error;
    }

    public double getPower(){
        double PID = Controller.calculate(slidesPosition, slidesTarget);
        return PID + f;
    }

    public void closeClaw(){
        clawServo.setPosition(1);
    }
    public void openClaw(){
        clawServo.setPosition(0);
    }
    public void activateClawAngler(){
        if(clawAngle>2.19) clawAnglerServo.setPosition(0.05);
        else if(clawAngle<2.18) clawAnglerServo.setPosition(0);
        clawServo.setPosition(0.5);
    }
    public double updateSlidePosition(){
        return (rightSlidesMotor.getCurrentPosition() + leftSlidesMotor.getCurrentPosition()) / 2.0;
    }
}
