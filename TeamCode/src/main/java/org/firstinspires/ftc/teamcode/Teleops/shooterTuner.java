package org.firstinspires.ftc.teamcode.Teleops;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name="shooterTuner", group="Linear OpMode")
public class shooterTuner extends OpMode {
    public DcMotorEx shooter1;
    public DcMotorEx shooter2;
    public DcMotorEx intakeMotor;
    public double highVelocity = 2000;
    public double lowVelocity = 1300;


    public double curTargetVelocity = highVelocity;


    double p = 0;
    double f = 0;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001};


    int stepIndex = 1;




    @Override
    public void init() {
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(p,0,0,f);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("done :)");
    }


    @Override
    public void loop() {
        if(gamepad1.yWasPressed()){
            if (curTargetVelocity == highVelocity)
            {
                curTargetVelocity = lowVelocity;
            }else{
                curTargetVelocity = highVelocity;
            }
        }


        if (gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        if(gamepad1.dpadLeftWasPressed()){
            f += stepSizes[stepIndex];
        }
        if(gamepad1.dpadRightWasPressed()){
            f -= stepSizes[stepIndex];
        }
        if(gamepad1.dpadUpWasPressed()){
            p += stepSizes[stepIndex];
        }
        if(gamepad1.dpadDownWasPressed()){
            p -= stepSizes[stepIndex];
        }




        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(p,0,0,f);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);


        shooter1.setVelocity(curTargetVelocity);
        shooter2.setVelocity(curTargetVelocity);
        double shooterVelocity = (shooter1.getVelocity() + shooter2.getVelocity())/2;
        double error = (curTargetVelocity - shooterVelocity);
        intakeMotor.setPower(gamepad1.left_stick_y);


        telemetry.addData("stepSize ()", stepSizes );
        telemetry.addData("Shooter Velocity", "%.4sf", shooterVelocity);
        telemetry.addData("error", "%.4sf", error );
        telemetry.addData("P (up/down)","%.4sf", p);
        telemetry.addData("F (left/right)", "%.4sf", f);








    }
}

