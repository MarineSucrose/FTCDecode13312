package org.firstinspires.ftc.teamcode.Autos;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Missile", group="Linear OpMode")
public class ballMissile extends LinearOpMode {
    //Motor Variables
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotor shooter1, shooter2;
    private DcMotor intakeMotor;
    private Servo blocker;
    private CRServo pivot;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()) {

            //Hardware Maps for motors
            leftFront = hardwareMap.get(DcMotor.class, "leftFront");
            leftBack = hardwareMap.get(DcMotor.class, "leftBack");
            rightFront = hardwareMap.get(DcMotor.class, "rightFront");
            rightBack = hardwareMap.get(DcMotor.class, "rightBack");

            pivot = hardwareMap.get(CRServo.class, "pivot");
            blocker = hardwareMap.get(Servo.class, "blocker");
            shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
            shooter2 = hardwareMap.get(DcMotor.class, "shooter2");
            intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

            rightFront.setDirection(DcMotor.Direction.REVERSE);
            rightBack.setDirection(DcMotor.Direction.REVERSE);
            shooter2.setDirection(DcMotor.Direction.REVERSE);
            intakeMotor.setDirection(DcMotor.Direction.REVERSE);


            blocker.setPosition(1);
            shooter1.setPower(0.9);
            shooter2.setPower(0.9);
            Thread.sleep(3000);
            blocker.setPosition(0);
            intakeMotor.setPower(-1);
            Thread.sleep(3000);
            leftFront.setPower(-1);
            rightFront.setPower(-1);
            leftBack.setPower(-1);
            rightBack.setPower(-1);
            Thread.sleep(500);
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
            shooter1.setPower(0);
            shooter2.setPower(0);
            Thread.sleep(60000);





        }
    }
}