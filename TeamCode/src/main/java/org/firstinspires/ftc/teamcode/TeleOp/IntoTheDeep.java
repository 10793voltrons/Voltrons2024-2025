package org.firstinspires.ftc.teamcode.TeleOp;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleUnaryOperator;

@TeleOp(name = "IntoTheDeepV2")

public class IntoTheDeep extends LinearOpMode {

    // Primero declaramos todas las variables que vamos a usar
    // ( Motores, servos y temporizadores)

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor arm;
    DcMotor slide2;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor slide;
    DcMotor jack;
    Servo lanzador;
    Servo wrist;
    Servo wall;
    Servo garra;

    public static double wristFront = 0.97;
    public static double wristMiddle = 0.85;
    public static double wristBack = 0.40;

    public static double wallGrab = 1;
    public static double wallDrop = 0.5;
    public static double linearSlidePowerMultiplier = 0.8;
    public static double servoCentered = 0.6;
    public static double servoleft = 0.3;
    public static double servoright = 0.9;
    public static double garraGrab = 0.6;
    public static double garraDrop = 0.3;

    //Timers de botones
    public static int cuelga = 0;


    ElapsedTime aButton = new ElapsedTime();
    ElapsedTime bButton = new ElapsedTime();
    ElapsedTime xButton = new ElapsedTime();
    ElapsedTime yButton = new ElapsedTime();
    ElapsedTime armDelay = new ElapsedTime();
    ElapsedTime leftBump2 = new ElapsedTime();
    ElapsedTime rightBump2 = new ElapsedTime();
    ElapsedTime dpad_up = new ElapsedTime();
    ElapsedTime dpad_down = new ElapsedTime();
    ElapsedTime dpad_left = new ElapsedTime();
    ElapsedTime dpad2_up = new ElapsedTime();
    ElapsedTime dpad2_down = new ElapsedTime();
    ElapsedTime dpad2_left = new ElapsedTime();
    ElapsedTime aButton2 = new ElapsedTime();
    ElapsedTime bButton2 = new ElapsedTime();
    ElapsedTime yButton2 = new ElapsedTime();
    ElapsedTime bButton1 = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        // Luego las asignamos a su respectivo pedazo de hardware
        leftFront = hardwareMap.dcMotor.get("fl");
        rightFront = hardwareMap.dcMotor.get("fr");
        leftBack = hardwareMap.dcMotor.get("bl");
        rightBack = hardwareMap.dcMotor.get("br");
        slide = hardwareMap.dcMotor.get("slide");
        arm = hardwareMap.dcMotor.get("arm");
        slide2 = hardwareMap.dcMotor.get("slide2");
        jack = hardwareMap.dcMotor.get("jack");
        wrist = hardwareMap.servo.get("wrist");
        wall = hardwareMap.servo.get("wall");
        garra = hardwareMap.servo.get("garra");
        lanzador = hardwareMap.servo.get("lanzador");


        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Invertimos los motores de fabrica
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        slide2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Hacemos esto para que por defecto, cuando alguien deje de mover el stick de motor, se frenen todos los motores y no se quede patinando
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        jack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Iniciamos los temporizadores
        aButton.reset();
        bButton.reset();
        xButton.reset();
        yButton.reset();
        aButton2.reset();
        bButton2.reset();
        yButton2.reset();
        armDelay.reset();
        leftBump2.reset();
        rightBump2.reset();
        dpad_up.reset();
        dpad_down.reset();
        dpad_left.reset();
        dpad2_up.reset();
        dpad2_down.reset();
        dpad2_left.reset();
        bButton1.reset();

        double invert = 1;
        double adjust = 10;
        int lanzado = 0;
        int wBajando = 0;
        int wAbierto1 = 0;
        int wPos = 0;
        int gAbierto1 = 0;


        wall.setPosition(wallGrab);
        wAbierto1 = 1;

        waitForStart();

        while (opModeIsActive()) {



            // Invert puede tener dos valores: 1 o -1. Al mutiplicar invert por eso, invertimos el poder que se le debe de asignar al motor
            // Adjust es un numero que se dividira entre 10, para generar un numero decimal (e.g. 0.5) entonces al multiplicar todo el valor por este, se reducira a la mitad el poder de las llantas

            rightFront.setPower((-gamepad1.left_stick_y/1.45 - gamepad1.left_stick_x/1.45 - (gamepad1.right_stick_x/1.45 * -invert)) * (adjust / 10.0));
            leftFront.setPower((-gamepad1.left_stick_y/1.45 + gamepad1.left_stick_x/1.45 + (gamepad1.right_stick_x/1.45 * -invert)) * (adjust / 10.0));
            rightBack.setPower((-gamepad1.left_stick_y/1.45 + gamepad1.left_stick_x/1.45 - (gamepad1.right_stick_x/1.45 * -invert)) * (adjust / 10.0));
            leftBack.setPower((-gamepad1.left_stick_y/1.45 - gamepad1.left_stick_x/1.45 + (gamepad1.right_stick_x/1.45 * -invert)) * (adjust / 10.0));

            // Slow Mode
            if (gamepad1.a && aButton.milliseconds() > 300) {
                if (adjust == 10) {
                    adjust = 4;
                } else {
                    adjust = 10;
                }
                aButton.reset();
            }

            // Invert Mode
            if (gamepad1.y && yButton.milliseconds() > 500) {
                if (invert == 1) {
                    leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
                    rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
                    leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
                    rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
                    invert = -1;
                } else {
                    leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
                    rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
                    leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
                    rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
                    invert = 1;
                }
                yButton.reset();
            }

            /** Mover slides para colgar **/
            if (gamepad1.b && cuelga == 0) {
                slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slide.setTargetPosition(-1300);
                slide2.setTargetPosition(-1300);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(0.8);
                slide2.setPower(0.8);
                slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                sleep(10000);
            }

            if (gamepad1.b && cuelga == 1) {
                slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            if (cuelga == 1 ){
                slide.setPower(-1);
                slide2.setPower(-1);
            }


            /** SERVO DE INTAKE **/
            if (gamepad2.y && yButton2.milliseconds()>100 ){
                lanzador.setPosition(servoCentered);
            }

            if (gamepad2.left_bumper && leftBump2.milliseconds()>100){
                lanzador.setPosition(servoleft);
            }

            if (gamepad2.right_bumper && rightBump2.milliseconds()>100){
                lanzador.setPosition(servoright);
            }

            /** SERVO QUE LEVANTA Y BAJA EL LANZADOR 3 POSICIONES**/
            if (gamepad2.dpad_down && dpad2_down.milliseconds() > 300) {
                wrist.setPosition(wristFront);
                dpad2_down.reset();
            }

            if (gamepad2.dpad_left && dpad2_left.milliseconds() > 300) {
                wrist.setPosition(wristMiddle);
            }

            if (gamepad2.dpad_up && dpad2_up.milliseconds() > 300) {
                wrist.setPosition(wristBack);
            }

            /** SERVO QUE AGARRA EL ESPECIMEN DE LA PARED **/
            if (gamepad2.b && bButton2.milliseconds() > 300) {
                if (wAbierto1==1){
                    wall.setPosition(wallDrop);
                    wAbierto1 = 0;
                }else{
                    wall.setPosition(wallGrab);
                    wAbierto1 = 1;
                }
            }
            if (gamepad2.a && aButton2.milliseconds() > 300) {
                if (gAbierto1==1){
                    garra.setPosition(garraDrop);
                    gAbierto1 = 0;
                }else{
                    garra.setPosition(garraGrab);
                    gAbierto1 = 1;
                }
            }

            /** MOTOR QUE SUBE Y BAJA LA COSA PARA COLGAR ESPECIMENES **/
            if(gamepad2.left_trigger>0)
            {
                slide.setPower(-linearSlidePowerMultiplier);
                slide2.setPower(-linearSlidePowerMultiplier);
            }
            else if(gamepad2.right_trigger>0)
            {
                slide.setPower(linearSlidePowerMultiplier*0.75);
                slide2.setPower(linearSlidePowerMultiplier*0.75);
            }
            else{
                slide.setPower(0);
                slide2.setPower(0);
            }

            if(gamepad1.dpad_up && dpad_up.milliseconds()>100 )
            {

                arm.setPower(-linearSlidePowerMultiplier);
            }
            else if(gamepad1.dpad_down && dpad_down.milliseconds()>100 )
            {
                arm.setPower(linearSlidePowerMultiplier*0.75);
            }
            else{
                arm.setPower(0);
            }

            /** Subir el jack para elevar el motor **/
            if(gamepad1.right_trigger>0)
            {
                jack.setPower(1);
            }else{

                jack.setPower(0);
            }
            if (gamepad1.left_trigger>0)
            {
                jack.setPower(-1);
            }else{

                jack.setPower(0);
            }



            telemetry.addData("Invert", invert);
            telemetry.addData("Wrist: ", wrist.getPosition());
            telemetry.addData("Slow Mode", adjust == 4);
            telemetry.addData("Trigger: ", gamepad2.right_trigger);
            telemetry.update();

        }
    }
}

