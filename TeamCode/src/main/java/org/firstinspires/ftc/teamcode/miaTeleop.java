package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

        @TeleOp(name="miaTeleop", group="Linear OpMode")
        public class miaTeleop extends LinearOpMode {
            //Motor Variables
            private DcMotor leftFront, leftBack, rightFront, rightBack;
            private DcMotor shooterPropel;
            private DcMotor intakeMotor;
            private CRServo servo1, servo2;

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


                    //response to buttons/human input

                    while (gamepad1.left_bumper) {
                        servo1.setPower(0.2);
                        servo2.setPower(-0.2);
                    }
                    while (gamepad1.right_bumper) {
                        servo1.setPower(-0.2);
                        servo2.setPower(0.2);
                    }
                    while (gamepad1.x){
                        intakeMotor.setPower(0.5);
                    }


                    if(gamepad1.a){
                        stateVar = !stateVar;
                    }
                    while(stateVar){
                        shooterPropel.setPower(0.9);
                    }

                }
            }
        }