package org.firstinspires.ftc.teamcode;
//test
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class miaTeleop {
        @TeleOp(name="MiasTeleop1", group="Linear OpMode")
        public class MiasTeleop1 extends LinearOpMode {
            //Motor Variables
            private DcMotor leftFront, leftBack, rightFront, rightBack;
            private DcMotor propelLeft, propelRight;
            private Servo servo1, servo2;

            private double driveSensitivity = 1;

            @Override
            public void runOpMode() throws InterruptedException {


                //Hardware Maps for motors
                leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
                leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
                rightFront  = hardwareMap.get(DcMotor.class, "rightFront");
                rightBack  = hardwareMap.get(DcMotor.class, "rightBack");


                double drivePower = -gamepad1.left_stick_y;
                double turnPower  =  gamepad1.right_stick_x;
                double strafePower = gamepad1.left_stick_x;

                double lfPower = Range.clip(drivePower + turnPower + strafePower, -driveSensitivity, driveSensitivity);
                double rfPower = Range.clip(drivePower - turnPower - strafePower,  -driveSensitivity, driveSensitivity);
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
                servo1 = hardwareMap.get(Servo.class, "servo1");
                servo2 = hardwareMap.get(Servo.class, "servo2");
                propelLeft = hardwareMap.get(DcMotor.class, "shooter1");
                propelRight = hardwareMap.get(DcMotor.class, "shooter2");

                propelRight.setDirection(DcMotor.Direction.REVERSE);

                waitForStart();
                //response to buttons/human input
                if (gamepad1.a)
                {
                    propelLeft.setPower(1);
                    propelRight.setPower(1);
                }
                if (gamepad1.left_trigger == 1.0)
                {
                    servo1.setPosition(0.5);
                    servo2.setPosition(0.5);
                }

                //assign power when button is pressed


            }
        }


    }


