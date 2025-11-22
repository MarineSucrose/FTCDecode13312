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
    private Servo kicker, pivot;

    private double driveSensitivity = 1;

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


            double drivePower = -gamepad1.left_stick_y;
            double turnPower = gamepad1.right_stick_x;
            double strafePower = gamepad1.left_stick_x;

            double lfPower = Range.clip(drivePower + turnPower + strafePower, -driveSensitivity, driveSensitivity);
            double rfPower = Range.clip(drivePower - turnPower - strafePower, -driveSensitivity, driveSensitivity);
            double lbPower = Range.clip(drivePower + turnPower - strafePower, -driveSensitivity, driveSensitivity);
            double rbPower = Range.clip(drivePower - turnPower + strafePower, -driveSensitivity, driveSensitivity);

            rightFront.setDirection(DcMotor.Direction.REVERSE);
            rightBack.setDirection(DcMotor.Direction.REVERSE);
            shooter2.setDirection(DcMotor.Direction.REVERSE);

            // Send calculated power to wheels
            leftFront.setPower(lfPower);
            leftBack.setPower(lbPower);
            rightFront.setPower(rfPower);
            rightBack.setPower(rbPower);


            //use of hardware map function to make a variable so we can manipulate for each motor and servo
            pivot = hardwareMap.get(Servo.class, "pivot");
            shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
            shooter2 = hardwareMap.get(DcMotor.class, "shooter2");
            intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
            kicker = hardwareMap.get(Servo.class, "kicker");



            //response to buttons/human input

           if(gamepad2.dpadUpWasPressed()){
               pivot.setPosition(1);
           }
            if(gamepad2.dpadLeftWasPressed()){
                pivot.setPosition(0.5);
            }
            if(gamepad2.dpadDownWasPressed()){
                pivot.setPosition(0);
            }

            //Shooter (toggle)

            if(gamepad2.a){
                shootToggle = !shootToggle;
            }

            if(shootToggle){
                shooter1.setPower(0.9);
                shooter2.setPower(0.9);
            } else {
                shooter1.setPower(0.0);
                shooter2.setPower(0.0);
            }



            // Intake

            intakeMotor.setPower(-1 * gamepad2.right_stick_y);


            //kicker

            if(gamepad2.x){
               kicker.setPosition(0.75);
            } else {
                kicker.setPosition(0);
            }

            }
        }
    }
