package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Config
@Autonomous(name= "Left_ITD", group = "drive")
public class Left_ITD extends LinearOpMode {

    DcMotor slide;
    DcMotor slide2;
    DcMotor jack;
    Servo wall;
    public static double wallGrab = 1;
    public static double wallDrop = 0.5;


    @Override
    public void runOpMode() throws InterruptedException {

        slide = hardwareMap.dcMotor.get("slide");
        slide2 = hardwareMap.dcMotor.get("slide2");
        jack = hardwareMap.dcMotor.get("jack");
        wall = hardwareMap.servo.get("wall");

        slide2.setDirection(DcMotorSimple.Direction.REVERSE);

        //slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wall.setPosition(wallGrab);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        /** Creando una posición de inicio**/
        Pose2d startPose = new Pose2d(-24, -60, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        /** Creando las trayectorias **/
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose) // Moverse al sumergible
                .strafeTo(new Vector2d(-5, -36))
                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end()) //NO SE USA
                .waitSeconds(1)
                .back(6.2)
                .build();

        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end()) //Separarse del sumergible
                //.waitSeconds(1)
                .forward(10)
                .build();

        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end()) //Moverse a la derecha para ir por Samples
                .strafeTo(new Vector2d(-36, -40))
                .build();

        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(traj4.end()) //Subir al tile central
                .back(27)  //Ajuste de distancia para llegar al primer sample
                .build();

        TrajectorySequence traj6 = drive.trajectorySequenceBuilder(traj5.end()) //Moverse a la derecha para empujar Sample
                .strafeRight(14)
                .build();

        TrajectorySequence traj7 = drive.trajectorySequenceBuilder(traj6.end()) //Empujar Sample
                .lineTo(new Vector2d(-51, -59))
                .build();

        TrajectorySequence traj8 = drive.trajectorySequenceBuilder(traj7.end()) // Regresar a empujar Sample Central
                .lineTo(new Vector2d(-51, -15))
                .build();

        TrajectorySequence traj9 = drive.trajectorySequenceBuilder(traj8.end()) //Colocándose para empujar el Sample Central
                .strafeTo(new Vector2d(-60, -15))
                .build();

        TrajectorySequence traj10 = drive.trajectorySequenceBuilder(traj9.end()) //Empujar Sample Central a Net Zone
                .lineTo(new Vector2d(-60,-52))
                .build();

        TrajectorySequence traj11 = drive.trajectorySequenceBuilder(traj10.end()) //Regresar por tercer Sample
                .lineTo(new Vector2d(-60,-15))
                .build();

        TrajectorySequence traj12 = drive.trajectorySequenceBuilder(traj11.end()) //Colocandose a empujar Sample
                .strafeTo(new Vector2d(-66, -15))
                .build();

        TrajectorySequence traj13 = drive.trajectorySequenceBuilder(traj12.end()) //Empujando Sample a Net Zone
                .lineTo(new Vector2d(-66,-49))
                .build();

        TrajectorySequence traj13_1 = drive.trajectorySequenceBuilder(traj13.end()) //Empujando Sample a Net Zone
                .lineTo(new Vector2d(-66,-25))
                .build();

        TrajectorySequence traj14 = drive.trajectorySequenceBuilder(traj13_1.end())
                .lineToLinearHeading(new Pose2d(-25,-15, Math.toRadians(180)))
                .build();

        TrajectorySequence traj15 = drive.trajectorySequenceBuilder(traj14.end())
                .strafeLeft(3)
                .build();

        /** cambio para el agarrar segundo specimen **/


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
        drive.setMotorPowers(-0.4,-0.4,-0.4,-0.4);
        sleep(600);
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

        slide.setTargetPosition(1300);
        slide2.setTargetPosition(1300);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(0.8);
        slide2.setPower(0.8);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wall.setPosition(wallDrop);
        sleep(100);
        //wall.setPosition(wallGrab);

        drive.followTrajectorySequence(traj4);
        drive.followTrajectorySequence(traj5);

        /**Mover el Jack para poder tocar la barra **/
        jack.setPower(0.5);
        sleep(500);

        drive.followTrajectorySequence(traj6);
        drive.followTrajectorySequence(traj7);
        drive.followTrajectorySequence(traj8);
        drive.followTrajectorySequence(traj9);
        drive.followTrajectorySequence(traj10);
        drive.followTrajectorySequence(traj11);
        drive.followTrajectorySequence(traj12);
        drive.followTrajectorySequence(traj13);
        drive.followTrajectorySequence(traj13_1);
        drive.followTrajectorySequence(traj14);
        drive.followTrajectorySequence(traj15);


        //wall.setPosition(wallGrab);
        slide.setTargetPosition(1300);
        slide2.setTargetPosition(1300);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(0.8);
        slide2.setPower(0.8);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /** Moverse opquito pal rfente **/
        drive.setMotorPowers(-0.4,-0.4,-0.4,-0.4);
        sleep(300);
        drive.setMotorPowers(0,0,0,0);
        sleep(100);

        telemetry.addData("slide pos: ", slide.getCurrentPosition());
        telemetry.addData("slide2 pos: ", slide2.getCurrentPosition());
        telemetry.update();
    }
}


