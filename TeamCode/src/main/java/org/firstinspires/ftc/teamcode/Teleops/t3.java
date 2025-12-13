package org.firstinspires.ftc.teamcode.Teleops;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="t3", group="Linear OpMode")
public class t3 extends LinearOpMode {
    //Motor Variables
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotor shooter1, shooter2;
    private DcMotor intakeMotor;
    private Servo blocker;
    private CRServo pivot;

    private double driveSensitivity = 1;
    private double precision = 1;
    private boolean shootToggle = false;

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

            double drivePower = -gamepad1.left_stick_y;
            double turnPower = -gamepad1.right_stick_x;
            double strafePower = gamepad1.left_stick_x;

            double lfPower = Range.clip(drivePower + turnPower + strafePower, -driveSensitivity, driveSensitivity);
            double rfPower = Range.clip(drivePower - turnPower - strafePower, -driveSensitivity, driveSensitivity);
            double lbPower = Range.clip(drivePower + turnPower - strafePower, -driveSensitivity, driveSensitivity);
            double rbPower = Range.clip(drivePower - turnPower + strafePower, -driveSensitivity, driveSensitivity);


            if (gamepad1.right_bumper){
                precision = 0.25;
            }

            if (gamepad1.left_bumper){
                precision = 1
            }


                leftFront.setPower(lfPower * precision);
                leftBack.setPower(lbPower * precision);
                rightFront.setPower(rfPower * precision);
                rightBack.setPower(rbPower * precision);






            //shooter angle adjusting

            pivot.setPower(-gamepad2.left_stick_y / 4);


            //blocker

            if(gamepad2.right_bumper){
                blocker.setPosition(1.0);
            }
            if(gamepad2.left_bumper){
                blocker.setPosition(0.0);
            }




            //Shooter (toggle)

            if(gamepad2.x){
                shootToggle = !shootToggle;
            }

            if(shootToggle){
                shooter1.setPower(0.75);
                shooter2.setPower(0.75);
            } else {
                shooter1.setPower(0.0);
                shooter2.setPower(0.0);
            }





            // Intake

            intakeMotor.setPower(-1 * gamepad2.right_stick_y);

            }
        }
    }