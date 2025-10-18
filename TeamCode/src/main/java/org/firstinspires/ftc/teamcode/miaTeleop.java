package org.firstinspires.ftc.teamcode;

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
            private DcMotor rotateMotor, propelMotor;
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

                // Send calculated power to wheels
                leftFront.setPower(lfPower);
                leftBack.setPower(lbPower);
                rightFront.setPower(rfPower);
                rightBack.setPower(rbPower);


                //use of hardware map function to make a variable so we can manipulate for each motor and servo
                servo1 = hardwareMap.get(Servo.class, "servo1");
                servo2 = hardwareMap.get(Servo.class, "servo2");
                propelMotor = hardwareMap.get(DcMotor.class, "shooter1");
                rotateMotor = hardwareMap.get(DcMotor.class, "shooter2");


                waitForStart();
                //response to buttons/human input
                if (gamepad1.a)
                {
                    rotateMotor.setPower(1);
                }
                if (gamepad1.b)
                {
                    propelMotor.setPower(1);
                }
                if (gamepad1.left_trigger == 1.0)
                {
                    servo1.setPosition(0.5);
                }
                if (gamepad1.right_trigger == 1.0)
                {
                    servo2.setPosition(0.1);
                }

                //assign power when button is pressed


            }
        }


    }


