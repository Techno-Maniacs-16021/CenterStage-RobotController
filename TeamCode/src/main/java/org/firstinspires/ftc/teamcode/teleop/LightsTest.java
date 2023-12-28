package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.bot.MecanumDrive;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp
public class LightsTest extends OpMode
{
    /*
     * Change the pattern every 10 seconds in AUTO mode.
     */
    private final static int LED_PERIOD = 3;

    /*
     * Rate limit gamepad button presses to every 500ms.
     */
    private final static int GAMEPAD_LOCKOUT = 0;

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    Telemetry.Item patternName;
    Telemetry.Item display;

    SampleRevBlinkinLedDriver.DisplayKind displayKind;
    Deadline ledCycleDeadline;
    Deadline gamepadRateLimit;

    protected enum DisplayKind {
        MANUAL,
        AUTO
    }
    private ElapsedTime loopTime = new ElapsedTime();
    private MecanumDrive drive;

    public static double speedMultiplier;
    @Override
    public void init(){

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));


        displayKind = SampleRevBlinkinLedDriver.DisplayKind.MANUAL;

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        blinkinLedDriver.setPattern(pattern);

        display = telemetry.addData("Display Kind: ", displayKind.toString());
        patternName = telemetry.addData("Pattern: ", pattern.toString());
        telemetry.update();

        ledCycleDeadline = new Deadline(LED_PERIOD, TimeUnit.SECONDS);
        gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);

    }
    @Override
    public void init_loop(){

    }
    @Override
    public void start(){

    }
    @Override
    public void loop(){
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

////////////////////////TELEMETRY////////////////////

/////////////////////////LED BLINKIN/////////////////////
        handleGamepad();

        if (displayKind == SampleRevBlinkinLedDriver.DisplayKind.AUTO) {
            doAutoDisplay();
        } else {
            /*
             * MANUAL mode: Nothing to do, setting the pattern as a result of a gamepad event.
             */
        }
    }

    /*
     * handleGamepad
     *
     * Responds to a gamepad button press.  Demonstrates rate limiting for
     * button presses.  If loop() is called every 10ms and and you don't rate
     * limit, then any given button press may register as multiple button presses,
     * which in this application is problematic.
     *
     * A: Manual mode, Right bumper displays the next pattern, left bumper displays the previous pattern.
     * B: Auto mode, pattern cycles, changing every LED_PERIOD seconds.
     */
    protected void handleGamepad()
    {
        if (!gamepadRateLimit.hasExpired()) {
            return;
        }

        if (gamepad1.a) {
            setDisplayKind(SampleRevBlinkinLedDriver.DisplayKind.MANUAL);
            gamepadRateLimit.reset();
        } else if (gamepad1.b) {
            setDisplayKind(SampleRevBlinkinLedDriver.DisplayKind.AUTO);
            gamepadRateLimit.reset();
        } else if ((displayKind == SampleRevBlinkinLedDriver.DisplayKind.MANUAL) && (gamepad1.left_bumper)) {
            pattern = pattern.previous();
            displayPattern();
            gamepadRateLimit.reset();
        } else if ((displayKind == SampleRevBlinkinLedDriver.DisplayKind.MANUAL) && (gamepad1.right_bumper)) {
            pattern = pattern.next();
            displayPattern();
            gamepadRateLimit.reset();
        } else if ((displayKind == SampleRevBlinkinLedDriver.DisplayKind.MANUAL) && (gamepad1.x)) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.CP1_2_NO_BLENDING;
            displayPattern();
            gamepadRateLimit.reset();
        } else if ((displayKind == SampleRevBlinkinLedDriver.DisplayKind.MANUAL) && (gamepad1.y)) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.CP1_2_SPARKLE_1_ON_2;
            displayPattern();
            gamepadRateLimit.reset();
        }

    }

    protected void setDisplayKind(SampleRevBlinkinLedDriver.DisplayKind displayKind)
    {
        this.displayKind = displayKind;
        display.setValue(displayKind.toString());
    }
    protected void doAutoDisplay()
    {
        if (ledCycleDeadline.hasExpired()) {
            if (pattern == RevBlinkinLedDriver.BlinkinPattern.AQUA) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
            }
            else {
                pattern = RevBlinkinLedDriver.BlinkinPattern.AQUA;
            }
            //pattern = pattern.next();
            displayPattern();
            ledCycleDeadline.reset();
        }
    }

    protected void displayPattern()
    {
        blinkinLedDriver.setPattern(pattern);
        patternName.setValue(pattern.toString());
    }
    @Override
    public void stop(){

    }
}
