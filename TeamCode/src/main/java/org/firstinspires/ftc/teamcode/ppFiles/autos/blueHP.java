package org.firstinspires.ftc.teamcode.ppFiles.autos;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ppFiles.Constants;


@Autonomous (name="BlueHP", group="Linear OpMode")
public class blueHP extends  LinearOpMode {

    private DcMotorEx shooter1, shooter2;
    private DcMotor intakeMotor;
    private Servo shootBlock;
    private Servo pivot;



    private Follower follower;
    private Timer opmodeTimer;


    //these are the different "states" the robot will be in, specific movements and actions
    private enum PathState {

        startPosToShootPos,
        shootPreload,

        toEndPose
    }

    PathState pathState;


    //all the poses the robot will be in when something happens
    private final Pose startPos = new Pose(60, 8, Math.toRadians(270));
    private final Pose shootPos = new Pose(60, 18, Math.toRadians(294));
    private final Pose humanPlayer = new Pose(2, 60, Math.toRadians(180));
    private final Pose endPose = new Pose(32, 12, Math.toRadians(90));


    //these are the paths the robot will follow, one pose to another
    private PathChain startPosToShootPos, toHP, returnShootPos, toEndPos;

    public void buildPaths() {
        startPosToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPos, shootPos))
                .setLinearHeadingInterpolation(startPos.getHeading(), shootPos.getHeading())
                .build();


        toHP = follower.pathBuilder()
            .addPath(new BezierLine(shootPos, humanPlayer))
            .setLinearHeadingInterpolation(shootPos.getHeading(), humanPlayer.getHeading())
            .build();


        returnShootPos = follower.pathBuilder()
                .addPath(new BezierLine(humanPlayer, shootPos))
                .setLinearHeadingInterpolation(shootPos.getHeading(), humanPlayer.getHeading())
                .build();


        toEndPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPos, endPose))
                .setLinearHeadingInterpolation(shootPos.getHeading(), endPose.getHeading())
                .build();


    }



    //this is what will activate each path and action in a sequence after it is called

    public void statePathUpdate() {
        switch (pathState) {



        }
    }





    public void shoot() {

        for (int i = 0; i <= 3; i++) {
            shootBlock.setPosition(0);

            intakeMotor.setPower(1);
            sleep(300);
            intakeMotor.setPower(0);

            if (i != 3) {
                sleep(500);
            }

            shootBlock.setPosition(0.5);

        }
    }






    @Override
    public void runOpMode() throws InterruptedException {
        pathState = PathState.startPosToShootPos;
        opmodeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        pivot = hardwareMap.get(Servo.class, "pivot");
        shootBlock = hardwareMap.get(Servo.class, "shootBlock");

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");


        shooter2.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(400, 0, 0, 5);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        buildPaths();
        follower.setPose(startPos);

        waitForStart();

        while (opModeIsActive()) {

            follower.update();
            statePathUpdate();

            shootBlock.setPosition(0.5);
            pivot.setPosition(1);
            shooter1.setVelocity(1975);
            shooter2.setVelocity(1975);


        }

    }
}
