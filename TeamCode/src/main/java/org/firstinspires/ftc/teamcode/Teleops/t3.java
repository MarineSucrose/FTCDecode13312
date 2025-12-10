package org.firstinspires.ftc.teamcode.Teleops;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="t3", group="Linear OpMode")
public class t3 extends LinearOpMode {
    //Motor Variables
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotor shooter1, shooter2;
    private DcMotor intakeMotor;
    private Servo pivot, blocker;

    private double driveSensitivity = 1;
    private boolean shootToggle = false;
    private boolean blockToggle = false;


    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()) {

            //Hardware Maps for motors
            leftFront = hardwareMap.get(DcMotor.class, "leftFront");
            leftBack = hardwareMap.get(DcMotor.class, "leftBack");
            rightFront = hardwareMap.get(DcMotor.class, "rightFront");
            rightBack = hardwareMap.get(DcMotor.class, "rightBack");

            pivot = hardwareMap.get(Servo.class, "pivot");
            blocker = hardwareMap.get(Servo.class, "blocker");
            shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
            shooter2 = hardwareMap.get(DcMotor.class, "shooter2");
            intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

            rightFront.setDirection(DcMotor.Direction.REVERSE);
            rightBack.setDirection(DcMotor.Direction.REVERSE);
            shooter2.setDirection(DcMotor.Direction.REVERSE);

            double drivePower = -gamepad1.left_stick_y;
            double turnPower = gamepad1.right_stick_x;
            double strafePower = gamepad1.left_stick_x;

            double lfPower = Range.clip(drivePower + turnPower + strafePower, -driveSensitivity, driveSensitivity);
            double rfPower = Range.clip(drivePower - turnPower - strafePower, -driveSensitivity, driveSensitivity);
            double lbPower = Range.clip(drivePower + turnPower - strafePower, -driveSensitivity, driveSensitivity);
            double rbPower = Range.clip(drivePower - turnPower + strafePower, -driveSensitivity, driveSensitivity);

            leftFront.setPower(lfPower);
            leftBack.setPower(lbPower);
            rightFront.setPower(rfPower);
            rightBack.setPower(rbPower);





            //shooter angle adjusting

            double pivPos = 0;

            if (gamepad2.a && pivPos <= 0.875){
                pivPos = pivPos + 0.125;
            }

            if (gamepad2.b && pivPos >= 0.125){
                pivPos = pivPos - 0.125;
            }

            pivot.setPosition(pivPos);




            //blocker (toggle)

            if(gamepad2.y){
                blockToggle = !blockToggle;
            }

            if(blockToggle){
                blocker.setPosition(0.0);
            } else {
                blocker.setPosition(0.5);
            }





            //Shooter (toggle)

            if(gamepad2.x){
                shootToggle = !shootToggle;
            }

            if(shootToggle){
                shooter1.setPower(0.8);
                shooter2.setPower(0.8);
            } else {
                shooter1.setPower(0.0);
                shooter2.setPower(0.0);
            }





            // Intake

            intakeMotor.setPower(-1 * gamepad2.right_stick_y);

            }
        }
    }