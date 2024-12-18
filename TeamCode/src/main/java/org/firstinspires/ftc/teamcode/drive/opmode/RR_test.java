package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

/*
 * This is an example of a more complex path to really test the tuning.
 */

@Disabled
@Autonomous(group = "drive")
public class RR_test extends LinearOpMode {

    DcMotor slide;
    Servo wall;
    public static double wallGrab = 1;
    public static double wallDrop = 0.5;


    @Override
    public void runOpMode() throws InterruptedException {

        slide = hardwareMap.dcMotor.get("slide");
        wall = hardwareMap.servo.get("wall");

        //slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wall.setPosition(wallGrab);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        /** Creando una posición de inicio**/
        Pose2d startPose = new Pose2d(24, -60, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        /** Creando las trayectorias **/
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(5, -36))
                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .waitSeconds(3)
                .back(6.2)
                .build();

        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .waitSeconds(3)
                .forward(10)
                .build();

        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())
                .strafeTo(new Vector2d(36, -40))
                .build();

        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(traj4.end())
                .back(36)
                .build();

        TrajectorySequence traj6 = drive.trajectorySequenceBuilder(traj5.end())
                .strafeLeft(10)
                .build();

        TrajectorySequence traj7 = drive.trajectorySequenceBuilder(traj6.end())
                .lineTo(new Vector2d(44, -50))
                .build();

        TrajectorySequence traj8 = drive.trajectorySequenceBuilder(traj7.end())
                .lineTo(new Vector2d(44, -10))
                .build();

        TrajectorySequence traj9 = drive.trajectorySequenceBuilder(traj8.end())
                .strafeTo(new Vector2d(58, -10))
                .build();

        TrajectorySequence traj10 = drive.trajectorySequenceBuilder(traj9.end())
                .lineTo(new Vector2d(58,-50))
                .build();

        TrajectorySequence traj11 = drive.trajectorySequenceBuilder(traj10.end())
                .lineTo(new Vector2d(58, -10))
                .build();

        TrajectorySequence traj12 = drive.trajectorySequenceBuilder(traj11.end())
                .lineToLinearHeading(new Pose2d(20, 0, Math.toRadians(0)))
                .build();

        waitForStart();

        if (isStopRequested()) {
            return;
        }
        drive.followTrajectorySequence(traj1);

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(1700);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(0.5);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sleep(2000);

        /** TODO: Agregar parte de la garra y colgar **/
        drive.setMotorPowers(-0.3,-0.3,-0.3,-0.3);
        sleep(800);
        drive.setMotorPowers(0,0,0,0);
        sleep(800);

        slide.setTargetPosition(2300);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(0.5);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sleep(1000);
        wall.setPosition(wallDrop);
        sleep(1000);

        drive.followTrajectorySequence(traj3);

        slide.setTargetPosition(1200);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(0.5);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wall.setPosition(wallGrab);

        drive.followTrajectorySequence(traj4);
        drive.followTrajectorySequence(traj5);
        drive.followTrajectorySequence(traj6);
        drive.followTrajectorySequence(traj7);
        drive.followTrajectorySequence(traj8);
        drive.followTrajectorySequence(traj9);
        drive.followTrajectorySequence(traj10);
        drive.followTrajectorySequence(traj11);
        drive.followTrajectorySequence(traj12);

        telemetry.addData("slide pos: ", slide.getCurrentPosition());
        telemetry.update();
    }
}


