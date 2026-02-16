package org.firstinspires.ftc.teamcode.Teleops;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="iris", group="Linear OpMode")
public class iris extends LinearOpMode {

    //Motor Variables
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotorEx shooter1, shooter2;
    private DcMotor intakeMotor;
    private Servo intakeBlock;
    private Servo shootBlock;
    private Servo pivot;

    private double driveSensitivity = 1;
    private double precision = 1;
    private double highVel = 2050;
    private double mediumVel = 1500;
    private double lowVel = 1300;
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

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(400, 0, 0, 5);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);


        while (opModeIsActive()) {

            double drivePower = -gamepad1.left_stick_y;
            double turnPower = gamepad1.right_stick_x;
            double strafePower = gamepad1.left_stick_x;

            double lfPower = Range.clip(drivePower + turnPower + strafePower, -driveSensitivity, driveSensitivity);
            double rfPower = Range.clip(drivePower - turnPower - strafePower, -driveSensitivity, driveSensitivity);
            double lbPower = Range.clip(drivePower + turnPower - strafePower, -driveSensitivity, driveSensitivity);
            double rbPower = Range.clip(drivePower - turnPower + strafePower, -driveSensitivity, driveSensitivity);

            if (gamepad1.right_bumper) {
                precision = 1;
            }

            if (gamepad1.left_bumper) {
                precision = 0.25;
            }

            leftFront.setPower(lfPower * precision);
            leftBack.setPower(lbPower * precision);
            rightFront.setPower(rfPower * precision);
            rightBack.setPower(rbPower * precision);


            //shooter angle adjusting

            if (gamepad2.dpad_down) {
                pivot.setPosition(0.25);
            }

            if (gamepad2.dpad_right) {
                pivot.setPosition(0.5);
            }

            if (gamepad2.dpad_up) {
                pivot.setPosition(0.75);
            }


            //blocker

            if (gamepad2.right_bumper) {
                shootBlock.setPosition(0.5);
            }
            if (gamepad2.left_bumper) {
                shootBlock.setPosition(0.0);
            }


            //Shooter (toggle)


            if (gamepad2.x) {
                curVel = 0;
            }

            if (gamepad2.b) {
                curVel = highVel;
                pivot.setPosition(0.5);
            }

            if (gamepad2.y) {
                curVel = mediumVel;
                pivot.setPosition(0.25);

            }

            if (gamepad2.a) {
                curVel = lowVel;
                pivot.setPosition(0.25);
            }

            shooter1.setVelocity(curVel);
            shooter2.setPower(curVel);


            // Intake
            intakeMotor.setPower(-1 * gamepad2.right_stick_y);

            if (gamepad2.right_trigger != 0) {
                intakeBlock.setPosition(0.0);
            } else {
                intakeBlock.setPosition(0.5);
            }

        }
    }
}