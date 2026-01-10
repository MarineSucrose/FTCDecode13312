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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.RoadrunnerFiles.MecanumDrive;




@Autonomous(name="redNear", group="Linear OpMode")
public class redNear extends LinearOpMode {


    private DcMotor shooter1, shooter2;
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
            shooter1.setPower(.7);
            shooter2.setPower(.7);
            sleep(2000);


        }
    }



    public class shoot3Artifacts implements InstantFunction{
        @Override
        public void run(){
            shootBlock.setPosition(0);

            intakeMotor.setPower(1);
            sleep(200);
            intakeMotor.setPower(0);
            sleep(1000);

            intakeMotor.setPower(1);
            sleep(200);
            intakeMotor.setPower(0);
            sleep(1000);

            intakeMotor.setPower(1);
            sleep(200);
            intakeMotor.setPower(0);
            sleep(1000);

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

        Pose2d beginPose = new Pose2d(-60, -60, Math.toRadians(135));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        pivot = hardwareMap.get(Servo.class, "pivot");
        shootBlock = hardwareMap.get(Servo.class, "shootBlock");
        intakeBlock = hardwareMap.get(Servo.class, "intakeBlock");

        shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");


        shooter2.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);





//---------------------
        waitForStart();

        Action path = drive.actionBuilder(beginPose)
                .stopAndAdd(new start())
                .strafeToLinearHeading(new Vector2d(-2, 12), Math.toRadians(135))
                .stopAndAdd(new shoot3Artifacts())
                .stopAndAdd(new intakeActivate())
                .splineTo(new Vector2d(54,12 ), Math.toRadians(180))
                .stopAndAdd(new intakeDeactivate())
                .splineTo(new Vector2d(54,12 ), Math.toRadians(135))
                .strafeToLinearHeading(new Vector2d(24, 0), Math.toRadians(135))
                .build();


        Actions.runBlocking(new SequentialAction(path));


    }

}

