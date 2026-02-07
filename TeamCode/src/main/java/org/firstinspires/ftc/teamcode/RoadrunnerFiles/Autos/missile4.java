package org.firstinspires.ftc.teamcode.RoadrunnerFiles.Autos;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="HailMary(SM4)", group="Linear OpMode")
public class missile4 extends LinearOpMode {

    //Motor Variables
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotorEx shooter1, shooter2;
    private DcMotor intakeMotor;
    private Servo intakeBlock;
    private Servo shootBlock;
    private Servo pivot;

    private double driveSensitivity = 1;
    private double precision = 1;
    private double extraHighVel = 2200;
    private double lowVel = 1250;
    private double curVel = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        //Hardware Maps for motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        pivot = hardwareMap.get(Servo.class, "pivot");
        shootBlock = hardwareMap.get(Servo.class, "shootBlock");
        intakeBlock = hardwareMap.get(Servo.class, "intakeBlock");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");


        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);


        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter2.setDirection(DcMotorEx.Direction.REVERSE);
        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(335, 0, 0, 25);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);


        while (opModeIsActive()) {

           pivot.setPosition(.75);
           shootBlock.setPosition(.5);
           shooter1.setVelocity(extraHighVel);
           shooter2.setVelocity(extraHighVel);
           sleep(2000);

           shootBlock.setPosition(0);
           intakeMotor.setPower(1);
           sleep(3000);

           leftFront.setPower(.5);
           leftBack.setPower(.5);
           rightFront.setPower(.5);
           rightBack.setPower(.5);

           sleep(2000);
           leftFront.setPower(.5);
           leftBack.setPower(.5);
           rightFront.setPower(.5);
           rightBack.setPower(.5);

            sleep(30000);


        }
    }
}