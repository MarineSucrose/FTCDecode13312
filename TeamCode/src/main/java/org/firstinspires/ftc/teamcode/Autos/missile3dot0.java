package org.firstinspires.ftc.teamcode.Autos;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Missile3.0", group="Linear OpMode")
public class missile3dot0 extends LinearOpMode {
    //Motor Variables
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotor shooter1, shooter2;
    private DcMotor intakeMotor;
    private Servo intakeBlock;
    private Servo shootBlock;
    private Servo pivot;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
            leftFront = hardwareMap.get(DcMotor.class, "leftFront");
            leftBack = hardwareMap.get(DcMotor.class, "leftBack");
            rightFront = hardwareMap.get(DcMotor.class, "rightFront");
            rightBack = hardwareMap.get(DcMotor.class, "rightBack");

            pivot = hardwareMap.get(Servo.class, "pivot");
            shootBlock = hardwareMap.get(Servo.class, "shootBlock");
            intakeBlock = hardwareMap.get(Servo.class, "intakeBlock");

            shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
            shooter2 = hardwareMap.get(DcMotor.class, "shooter2");
            intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

            leftFront.setDirection(DcMotor.Direction.REVERSE);
            leftBack.setDirection(DcMotor.Direction.REVERSE);
            shooter2.setDirection(DcMotor.Direction.REVERSE);
            intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        while (opModeIsActive()) {

            shootBlock.setPosition(0);
            shooter1.setPower(.9);
            shooter2.setPower(.9);

            intakeMotor.setPower(1);
            sleep(200);
            intakeMotor.setPower(0);
            sleep(1000);

            intakeMotor.setPower(1);
            sleep(200);
            intakeMotor.setPower(0);
            sleep(1000);


            intakeMotor.setPower(1);
            sleep(200);
            intakeMotor.setPower(0);
            sleep(1000);


            rightFront.setPower(0.5);
            leftFront.setPower(0.5);
            rightBack.setPower(0.5);
            leftBack.setPower(0.5);

            sleep(1000);

            rightFront.setPower(0);
            leftFront.setPower(0);
            rightBack.setPower(0);
            leftBack.setPower(0);

            sleep(100000);




        }
    }
}