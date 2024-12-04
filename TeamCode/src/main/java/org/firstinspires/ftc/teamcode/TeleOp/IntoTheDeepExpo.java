package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "IntoTheDeepExpo")

public class IntoTheDeepExpo extends LinearOpMode {

    // Primero declaramos todas las variables que vamos a usar
    // ( Motores, servos y temporizadores)

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor arm;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor slide;
    Servo lanzador;
    Servo wrist;
    Servo wall;

    public static double wristFront = 1.0;
    public static double wristMiddle = 0.90;
    public static double wristBack = 0.40;

    public static double wallGrab = 1;
    public static double wallDrop = 0.5;
    public static double linearSlidePowerMultiplier = 0.6;

    ElapsedTime aButton = new ElapsedTime();
    ElapsedTime bButton = new ElapsedTime();
    ElapsedTime xButton = new ElapsedTime();
    ElapsedTime yButton = new ElapsedTime();
    ElapsedTime armDelay = new ElapsedTime();
    ElapsedTime bump2 = new ElapsedTime();
    ElapsedTime dpad_up = new ElapsedTime();
    ElapsedTime dpad_down = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        // Luego las asignamos a su respectivo pedazo de hardware
        leftFront = hardwareMap.dcMotor.get("fl");
        rightFront = hardwareMap.dcMotor.get("fr");
        leftBack = hardwareMap.dcMotor.get("bl");
        rightBack = hardwareMap.dcMotor.get("br");
        slide = hardwareMap.dcMotor.get("slide");
        arm = hardwareMap.dcMotor.get("arm");
        wrist = hardwareMap.servo.get("wrist");
        wall = hardwareMap.servo.get("wall");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Invertimos los motores de fabrica
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        // Hacemos esto para que por defecto, cuando alguien deje de mover el stick de motor, se frenen todos los motores y no se quede patinando
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lanzador = hardwareMap.servo.get("lanzador");

        // Reiniciamos los temporizadores
        aButton.reset();
        xButton.reset();
        yButton.reset();
        bButton.reset();
        armDelay.reset();
        bump2.reset();
        dpad_up.reset();
        dpad_down.reset();

        double invert = 1;
        double adjust = 4;
        int lanzado = 0;
        int wBajando = 0;
        int wAbierto1 = 0;
        int wPos = 0;

        waitForStart();
        while (opModeIsActive()) {

            // Invert puede tener dos valores: 1 o -1. Al mutiplicar invert por eso, invertimos el poder que se le debe de asignar al motor
            // Adjust es un numero que se dividira entre 10, para generar un numero decimal (e.g. 0.5) entonces al multiplicar todo el valor por este, se reducira a la mitad el poder de las llantas

            rightFront.setPower((-gamepad1.left_stick_y/1.45 - gamepad1.left_stick_x/1.45 - (gamepad1.right_stick_x/1.45 * -invert)) * (adjust / 10.0));
            leftFront.setPower((-gamepad1.left_stick_y/1.45 + gamepad1.left_stick_x/1.45 + (gamepad1.right_stick_x/1.45 * -invert)) * (adjust / 10.0));
            rightBack.setPower((-gamepad1.left_stick_y/1.45 + gamepad1.left_stick_x/1.45 - (gamepad1.right_stick_x/1.45 * -invert)) * (adjust / 10.0));
            leftBack.setPower((-gamepad1.left_stick_y/1.45 - gamepad1.left_stick_x/1.45 + (gamepad1.right_stick_x/1.45 * -invert)) * (adjust / 10.0));


            /** SERVO QUE LEVANTA Y BAJA EL LANZADOR 3 POSICIONES**/
            if (gamepad1.a && aButton.milliseconds() > 300) {
                wrist.setPosition(wristFront);
                bump2.reset();
            }

            if (gamepad1.x && xButton.milliseconds() > 300) {
                wrist.setPosition(wristMiddle);
            }

            if (gamepad1.y && yButton.milliseconds() > 300) {
                wrist.setPosition(wristBack);
            }

            /** SERVO QUE AGARRA EL ESPECIMEN DE LA PARED **/
            if (gamepad1.b && bButton.milliseconds() > 300) {
                if (wAbierto1==1){
                    wall.setPosition(wallDrop);
                    wAbierto1 = 0;
                }else{
                    wall.setPosition(wallGrab);
                    wAbierto1 = 1;
                }
            }

            /** MOTOR QUE SUBE Y BAJA LA COSA PARA COLGAR ESPECIMENES **/
            if(gamepad1.left_trigger>0 /*&& DRight.milliseconds()>100*/ )
            {
                slide.setPower(-linearSlidePowerMultiplier);
            }
            else if(gamepad1.right_trigger>0 /*&& DLeft.milliseconds()>100*/ )
            {
                slide.setPower(linearSlidePowerMultiplier*0.75);
            }
            else{
                slide.setPower(0);
            }

            telemetry.addData("Garra: ", wrist.getPosition());
            telemetry.update();

        }
    }
}

