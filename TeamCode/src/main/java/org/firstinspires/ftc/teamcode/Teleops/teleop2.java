package org.firstinspires.ftc.teamcode.Teleops;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="teleop2", group="Linear OpMode")
public class teleop2 extends LinearOpMode {
    //Motor Variables
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotor shooterPropel;
    private DcMotor intakeMotor;
    private CRServo servo1, servo2;
    private Servo kicker;

    private double driveSensitivity = 1;

    private boolean stateVar = false;

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

            // Send calculated power to wheels
            leftFront.setPower(lfPower);
            leftBack.setPower(lbPower);
            rightFront.setPower(rfPower);
            rightBack.setPower(rbPower);


            //use of hardware map function to make a variable so we can manipulate for each motor and servo
            servo1 = hardwareMap.get(CRServo.class, "servo1");
            servo2 = hardwareMap.get(CRServo.class, "servo2");
            shooterPropel = hardwareMap.get(DcMotor.class, "shooterPropel");
            intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
            kicker = hardwareMap.get(Servo.class, "kicker");



            //response to buttons/human input

            //Shooter Angle Adjust


                servo1.setPower(0.2 * gamepad2.left_stick_y);
                servo2.setPower(-0.2 * gamepad2.left_stick_y);




            //Shooter (toggle)

            if(gamepad2.a){
                stateVar = !stateVar;
            }

            if(stateVar){
                shooterPropel.setPower(0.9);
            } else {
                shooterPropel.setPower(0.0);
            }



            // Intake

            intakeMotor.setPower(-1 * gamepad2.right_stick_y);


            //kicker

            while(gamepad2.x){
                kicker.setPosition(0.75);
            }
            while(gamepad2.b){
                kicker.setPosition(0);
            }
        }
    }
}
