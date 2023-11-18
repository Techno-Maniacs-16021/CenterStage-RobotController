package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.MecanumDrive;

/*
 * This is a simple routine to test turning capabilities.
 */
@Autonomous
@Config
public class StrafeLeftFutherSide extends LinearOpMode {

    public static double DISTANCE = 24; //inches

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-40,64,0));

        waitForStart();
/*
        if (isStopRequested()) return;
        new TrajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(64, 64, Math.toRadians(180)))
                .build();
*/

        drive.leftBack.setPower(1);
        drive.rightBack.setPower(1);
        drive.rightFront.setPower(1);
        drive.leftFront.setPower(1);
        sleep(1000);
        drive.leftBack.setPower(1);
        drive.leftFront.setPower(-1);
        drive.rightFront.setPower(1);
        drive.rightBack.setPower(-1);
        sleep(5000);
        drive.rightBack.setPower(0);
        drive.rightFront.setPower(0);
        drive.leftFront.setPower(0);
        drive.leftBack.setPower(0);


    }
}
