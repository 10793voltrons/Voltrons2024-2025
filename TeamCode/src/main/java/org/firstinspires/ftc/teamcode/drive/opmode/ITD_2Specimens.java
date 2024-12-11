package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Config
@Autonomous(group = "drive")
public class ITD_2Specimens extends LinearOpMode {

    DcMotor slide;
    DcMotor slide2;
    Servo wall;
    public static double wallGrab = 1;
    public static double wallDrop = 0.5;


    @Override
    public void runOpMode() throws InterruptedException {

        slide = hardwareMap.dcMotor.get("slide");
        slide2 = hardwareMap.dcMotor.get("slide2");
        wall = hardwareMap.servo.get("wall");

        slide2.setDirection(DcMotorSimple.Direction.REVERSE);

        //slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wall.setPosition(wallGrab);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        /** Creando una posición de inicio**/
        Pose2d startPose = new Pose2d(24, -60, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        /** Creando las trayectorias **/
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(3, -36))
                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .waitSeconds(1)
                .back(6.2)
                .build();

        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .waitSeconds(1)
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
                .lineTo(new Vector2d(48, -50))
                .build();

        TrajectorySequence traj8 = drive.trajectorySequenceBuilder(traj7.end())
                .lineTo(new Vector2d(48, -10))
                .build();

        TrajectorySequence traj9 = drive.trajectorySequenceBuilder(traj8.end())
                .strafeTo(new Vector2d(60, -10))
                .build();

        TrajectorySequence traj10 = drive.trajectorySequenceBuilder(traj9.end())
                .lineTo(new Vector2d(60,-50))
                .build();
        /** cambio para el agarrar segundo specimen **/

        TrajectorySequence traj11 = drive.trajectorySequenceBuilder(traj10.end())
                .lineToLinearHeading(new Pose2d(50,-45, Math.toRadians(90)))
                .build();

        /** Movimientos para ir a colgarlo **/

        TrajectorySequence traj12 = drive.trajectorySequenceBuilder(traj11.end())
                .lineToLinearHeading(new Pose2d(-3,-36, Math.toRadians(-90)))
                .build();

        waitForStart();

        if (isStopRequested()) {
            return;
        }
        drive.followTrajectorySequence(traj1);

        /** Levanta el slide a posición de 2da barra**/
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(1700);
        slide2.setTargetPosition(1700);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(0.8);
        slide2.setPower(0.8);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sleep(1200);

        /** TODO: Agregar parte de la garra y colgar **/
        drive.setMotorPowers(-0.3,-0.3,-0.3,-0.3);
        sleep(800);
        drive.setMotorPowers(0,0,0,0);
        sleep(200);

        /** Colgar el especimen **/
        slide.setTargetPosition(2300);
        slide2.setTargetPosition(2300);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(0.8);
        slide2.setPower(0.8);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sleep(400);
        wall.setPosition(wallDrop);
        sleep(100);

        drive.followTrajectorySequence(traj3);

        /** Bajar los slide **/

        slide.setTargetPosition(0);
        slide2.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(0.8);
        slide2.setPower(0.8);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //wall.setPosition(wallGrab);

        drive.followTrajectorySequence(traj4);
        drive.followTrajectorySequence(traj5);
        drive.followTrajectorySequence(traj6);
        drive.followTrajectorySequence(traj7);
        drive.followTrajectorySequence(traj8);
        drive.followTrajectorySequence(traj9);
        drive.followTrajectorySequence(traj10);
        drive.followTrajectorySequence(traj11);

        /** Se mueve a agarrar el especimen en el perímetro **/
        drive.setMotorPowers(-0.4,-0.4,-0.4,-0.4);
        sleep(800);
        drive.setMotorPowers(0,0,0,0);
        sleep(100);
        wall.setPosition(wallGrab);
        sleep(600);

        /** levanta especimen para colgarlo **/
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(2300);
        slide2.setTargetPosition(2300);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(0.8);
        slide2.setPower(0.8);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sleep(500);
        drive.followTrajectorySequence(traj12);

        /** Acercarse a colgar el especimen **/
        drive.setMotorPowers(-0.4,-0.4,-0.4,-0.4);
        sleep(800);
        drive.setMotorPowers(0,0,0,0);
        sleep(200);

        /** Colgar el especimen **/
        slide.setTargetPosition(1300);
        slide2.setTargetPosition(1300);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(0.8);
        slide2.setPower(0.8);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sleep(300);
        wall.setPosition(wallDrop);
        sleep(100);

        drive.setMotorPowers(-0.4,-0.4,-0.4,-0.4);
        sleep(200);
        drive.setMotorPowers(0,0,0,0);
        sleep(50);





        telemetry.addData("slide pos: ", slide.getCurrentPosition());
        telemetry.addData("slide2 pos: ", slide2.getCurrentPosition());
        telemetry.update();
    }
}


