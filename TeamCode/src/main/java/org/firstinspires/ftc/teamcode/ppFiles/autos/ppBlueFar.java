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


@Autonomous (name="BlueFar", group="Linear OpMode")
public class ppBlueFar extends  LinearOpMode {

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
        prepPickup1,
        pickup1,
        returnShootPos1,
        shootRound1,
        prepPickup2,
        pickup2,
        returnShootPos2,
        shootRound2,
        toEndPose
    }

    PathState pathState;


    //all the poses the robot will be in when something happens
    private final Pose startPos = new Pose(60, 8, Math.toRadians(270));
    private final Pose shootPos = new Pose(60, 18, Math.toRadians(294));

    private final Pose prepPickup1 = new Pose(48, 35, Math.toRadians(180));
    private final Pose pickup1 = new Pose(2, 35, Math.toRadians(180));

    private final Pose prepPickup2 = new Pose(48, 60, Math.toRadians(180));
    private final Pose pickup2 = new Pose(2, 60, Math.toRadians(180));

    private final Pose endPose = new Pose(32, 12, Math.toRadians(90));


    //these are the paths the robot will follow, one pose to another
    private PathChain startPosToShootPos, shootPosToPrepP1, prepP1ToP1, returnShootPos1, shootPosToPrepP2, prepP2ToP2, returnShootPos2, toEndPos;

    public void buildPaths() {
        startPosToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPos, shootPos))
                .setLinearHeadingInterpolation(startPos.getHeading(), shootPos.getHeading())
                .build();


        shootPosToPrepP1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPos, prepPickup1))
                .setLinearHeadingInterpolation(shootPos.getHeading(), prepPickup1.getHeading())
                .build();

        prepP1ToP1 = follower.pathBuilder()
                .addPath(new BezierLine(prepPickup1, pickup1))
                .setTangentHeadingInterpolation()
                .build();

        returnShootPos1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1, shootPos))
                .setLinearHeadingInterpolation(pickup1.getHeading(), shootPos.getHeading())
                .build();

        shootPosToPrepP2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPos, prepPickup2))
                .setLinearHeadingInterpolation(shootPos.getHeading(), prepPickup2.getHeading())
                .build();

        prepP2ToP2 = follower.pathBuilder()
                .addPath(new BezierLine(prepPickup2, pickup2))
                .setTangentHeadingInterpolation()
                .build();

        returnShootPos2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2, shootPos))
                .setLinearHeadingInterpolation(pickup2.getHeading(), shootPos.getHeading())
                .build();

        toEndPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPos, endPose))
                .setLinearHeadingInterpolation(shootPos.getHeading(), endPose.getHeading())
                .build();


    }



    //this is what will activate each path and action in a sequence after it is called

    public void statePathUpdate() {
        switch (pathState) {

            case startPosToShootPos:
                if (!follower.isBusy()) {
                    follower.followPath(startPosToShootPos, true);
                    pathState = PathState.shootPreload;
                }

            case shootPreload:
                if (!follower.isBusy()) {
                    sleep(1000);
                    shoot();
                    pathState = PathState.prepPickup1;
                }

            case prepPickup1:
                if (!follower.isBusy()) {
                    intakeMotor.setPower(1);
                    follower.followPath(shootPosToPrepP1, true);
                    pathState = PathState.pickup1;
                }

            case pickup1:
                if (!follower.isBusy()) {
                    follower.followPath(prepP1ToP1, true);
                    pathState = PathState.returnShootPos1;
                }

            case returnShootPos1:
                if (!follower.isBusy()) {
                    intakeMotor.setPower(0);
                    follower.followPath(returnShootPos1, true);
                    pathState = PathState.shootRound1;
                }


            case shootRound1:
                if (!follower.isBusy()) {
                   shoot();
                    pathState = PathState.prepPickup2;
                }




            case prepPickup2:
                if (!follower.isBusy()) {
                    intakeMotor.setPower(1);
                    follower.followPath(shootPosToPrepP2, true);
                    pathState = PathState.pickup2;
                }

            case pickup2:
                if (!follower.isBusy()) {
                    follower.followPath(prepP2ToP2, true);
                    pathState = PathState.returnShootPos2;
                }

            case returnShootPos2:
                if (!follower.isBusy()) {
                    intakeMotor.setPower(0);
                    follower.followPath(returnShootPos2, true);
                    pathState = PathState.shootRound2;
                }


            case shootRound2:
                if (!follower.isBusy()) {
                    shoot();
                    pathState = PathState.toEndPose;
                }

            case toEndPose:
                if (!follower.isBusy()) {
                    follower.followPath(toEndPos, true);
                }

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
