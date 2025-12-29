package org.firstinspires.ftc.teamcode.Teleops;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="iris", group="Linear OpMode")
public class iris extends LinearOpMode {

    //Motor Variables
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotor shooter1, shooter2;
    private DcMotor intakeMotor;
    private Servo intakeBlock;
    private Servo shootBlock;
    private Servo pivot;

    private double driveSensitivity = 1;
    private double precision = 1;
    private double strength = 0;

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

            shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
            shooter2 = hardwareMap.get(DcMotor.class, "shooter2");
            intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");


            rightFront.setDirection(DcMotor.Direction.REVERSE);
            rightBack.setDirection(DcMotor.Direction.REVERSE);
            shooter2.setDirection(DcMotor.Direction.REVERSE);
            intakeMotor.setDirection(DcMotor.Direction.REVERSE);



            while (opModeIsActive()) {

            double drivePower = -gamepad1.left_stick_y;
            double turnPower = -gamepad1.right_stick_x;
            double strafePower = gamepad1.left_stick_x;

            double lfPower = Range.clip(drivePower + turnPower + strafePower, -driveSensitivity, driveSensitivity);
            double rfPower = Range.clip(drivePower - turnPower - strafePower, -driveSensitivity, driveSensitivity);
            double lbPower = Range.clip(drivePower + turnPower - strafePower, -driveSensitivity, driveSensitivity);
            double rbPower = Range.clip(drivePower - turnPower + strafePower, -driveSensitivity, driveSensitivity);

            if(gamepad1.right_bumper){
                precision = 1;
            }

            if(gamepad1.left_bumper){
                precision = 0.25;
            }

            leftFront.setPower(lfPower * precision);
            leftBack.setPower(lbPower * precision);
            rightFront.setPower(rfPower * precision);
            rightBack.setPower(rbPower * precision);




            //shooter angle adjusting

            if(gamepad2.dpad_down) {
                pivot.setPosition(0.25);
            }

            if(gamepad2.dpad_right) {
                pivot.setPosition(0.5);
            }

            if(gamepad2.dpad_up) {
                pivot.setPosition(0.75);
            }


            //blocker

            if(gamepad2.right_bumper){
                shootBlock.setPosition(1.0);
            }
            if(gamepad2.left_bumper){
                shootBlock.setPosition(0.0);
            }


            //Shooter (toggle)

            if(gamepad2.y){
                strength = -0.2;
            }

            if(gamepad2.x){
                strength = 0;
            }

            if(gamepad2.b){
                strength = 0.9;
            }

            if(gamepad2.a){
                strength = 0.7;
            }

                shooter1.setPower(strength);
                shooter2.setPower(strength);





            // Intake
            intakeMotor.setPower(-1 * gamepad2.right_stick_y);

            if(gamepad2.right_trigger != 0) {
                intakeBlock.setPosition(0.0);
            } else {
                intakeBlock.setPosition(0.5);
            }

            }

        }

    }