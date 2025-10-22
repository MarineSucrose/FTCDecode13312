package org.firstinspires.ftc.teamcode;
//test
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class miaTeleop extends LinearOpMode {

    //Programming
    //revise code regarding hardware to reflect changes in bot (eliminate 2nd shooter motor, switch servos to axon servos, etc)
    //Finish configuration for shooter after revising code
    //brief drive team on controls and procedures so they can practice
    //finish auto 1
            //Motor Variables
            private DcMotor leftFront, leftBack, rightFront, rightBack;
            private DcMotor shooterMotor;
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
                shooterMotor = hardwareMap.get(DcMotor.class, "shooter1");


                waitForStart();
                //response to buttons/human input
                if (gamepad1.a)
                {
                    //1 is full power
                    shooterMotor.setPower(0.4);

                }
                if (gamepad1.left_trigger == 1.0)
                {
                    // 0 is 0 degrees, 0.5 is 90, 1 is 180
                    servo1.setPosition(0.5);
                    servo2.setPosition(0.5);
                }

                //assign power when button is pressed


            }



    }


