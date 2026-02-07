package org.firstinspires.ftc.teamcode.Autos;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.RoadrunnerFiles.MecanumDrive;




@Autonomous(name="blueFar", group="Linear OpMode")
public class blueFar extends LinearOpMode {

    private DcMotorEx shooter1, shooter2;
    private DcMotor intakeMotor;
    private Servo intakeBlock;
    private Servo shootBlock;
    private Servo pivot;




    //CLASSES FOR ACTIONS
    //--------------------
    public class start implements InstantFunction {
        @Override
        public void run() {
            shootBlock.setPosition(0.5);
            pivot.setPosition(0.75);
            shooter1.setVelocity(2025);
            shooter2.setVelocity(2025);
            sleep(2000);


        }
    }



    public class shoot3Artifacts implements InstantFunction{
        @Override
        public void run(){
            shootBlock.setPosition(0);

            intakeMotor.setPower(1);
            sleep(2000);
            intakeMotor.setPower(0);
            shootBlock.setPosition(0.5);



        }
    }


    public class intakeActivate implements InstantFunction {
        @Override
        public void run() {
            intakeMotor.setPower(1);
        }
    }


    public class intakeDeactivate implements InstantFunction {
        @Override
        public void run() {
            intakeMotor.setPower(0);
        }
    }

    //-------------------




    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d beginPose = new Pose2d(-6, -72, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        pivot = hardwareMap.get(Servo.class, "pivot");
        shootBlock = hardwareMap.get(Servo.class, "shootBlock");
        intakeBlock = hardwareMap.get(Servo.class, "intakeBlock");

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");


        shooter2.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);


        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(335, 0, 0, 25);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);



//---------------------
        waitForStart();

        Action path = drive.actionBuilder(beginPose)
                .stopAndAdd(new start())
                .strafeToLinearHeading(new Vector2d(-18, -60), Math.toRadians(90))
                .turnTo(Math.toRadians(280))
                .stopAndAdd(new shoot3Artifacts())
                .stopAndAdd(new intakeActivate())
                .strafeToLinearHeading(new Vector2d(-18, -36), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-48, -36), Math.toRadians(180))
                .stopAndAdd(new intakeDeactivate())
                .strafeToLinearHeading(new Vector2d(-18, -60), Math.toRadians(290))
                .stopAndAdd(new shoot3Artifacts())
                .strafeToLinearHeading(new Vector2d(-48, -24), 90)
                .build();


        Actions.runBlocking(new SequentialAction(path));


    }

}