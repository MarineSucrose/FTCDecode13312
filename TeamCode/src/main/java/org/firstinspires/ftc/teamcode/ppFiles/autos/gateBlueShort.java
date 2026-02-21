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


@Autonomous(name="gateBlueShort", group="Linear OpMode")
public class gateBlueShort extends  LinearOpMode {


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
        prepOpenGate,
        openGate,

        returnShootPos1,
        shootR1,

        prepPickup2,
        pickup2,
        returnShootPos2,
        shootR2,



        toEndPose

    }


    PathState pathState;


    //all the poses the robot will be in when something happens
    private final Pose startPos = new Pose(19, 124, Math.toRadians(325));
    private final Pose shootPos = new Pose(60, 90, Math.toRadians(320));

    private final Pose prepPickup2 = new Pose(48, 84, Math.toRadians(180));
    private final Pose pickup2 = new Pose(16, 84, Math.toRadians(180));

    private final Pose prepPickup1 = new Pose(48, 60, Math.toRadians(180));
    private final Pose pickup1 = new Pose(16, 60, Math.toRadians(180));

    private final Pose gate = new Pose (20, 70, Math.toRadians(180));

    private final Pose endPose = new Pose(60, 120, Math.toRadians(270));



    //these are the paths the robot will follow, one pose to another
    private PathChain startPosToShootPos, toPrepP1, toP1, toPrepGate, toGate, returnShootPos1, toPrepP2, toP2, returnShootPos2, toEndPos;


    public void buildPaths() {
        startPosToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPos, shootPos))
                .setLinearHeadingInterpolation(startPos.getHeading(), shootPos.getHeading())
                .build();

        toPrepP1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPos, prepPickup1))
                .setLinearHeadingInterpolation(shootPos.getHeading(), prepPickup1.getHeading())
                .build();

        toP1 = follower.pathBuilder()
                .addPath(new BezierLine(prepPickup1, pickup1))
                .setLinearHeadingInterpolation(prepPickup1.getHeading(), pickup1.getHeading())
                .build();

        toPrepGate = follower.pathBuilder()
                .addPath(new BezierLine(pickup1, prepPickup1))
                .setLinearHeadingInterpolation(pickup1.getHeading(), prepPickup1.getHeading())
                .build();

        toGate = follower.pathBuilder()
                .addPath(new BezierLine(prepPickup1, gate))
                .setLinearHeadingInterpolation(prepPickup1.getHeading(), gate.getHeading())
                .build();

        returnShootPos1 = follower.pathBuilder()
                .addPath(new BezierLine(gate, shootPos))
                .setLinearHeadingInterpolation(gate.getHeading(), shootPos.getHeading())
                .build();

        toPrepP2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPos, prepPickup2))
                .setLinearHeadingInterpolation(shootPos.getHeading(), prepPickup2.getHeading())
                .build();

        toP2 = follower.pathBuilder()
                .addPath(new BezierLine(prepPickup2, pickup2))
                .setLinearHeadingInterpolation(prepPickup2.getHeading(), pickup2.getHeading())
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

                    shoot();

                    pathState = PathState.prepPickup1;
                }

            case prepPickup1:
                if (!follower.isBusy()) {

                    intakeMotor.setPower(1);

                    follower.followPath(toPrepP1, true);
                    pathState = PathState.pickup1;

                }

            case pickup1:
                if (!follower.isBusy()) {
                    follower.followPath(toP1, true);
                    pathState = PathState.prepOpenGate;

                }

            case prepOpenGate:
                if (!follower.isBusy()) {

                    intakeMotor.setPower(0);

                    follower.followPath(toPrepGate, true);
                    pathState = PathState.openGate;

                }

            case openGate:
                if (!follower.isBusy()) {
                    follower.followPath(toGate, false);
                    pathState = PathState.returnShootPos1;

                }

            case returnShootPos1:
                if (!follower.isBusy()) {
                    follower.followPath(returnShootPos1, true);
                    pathState = PathState.shootR1;

                }

            case shootR1:
                if (!follower.isBusy()) {
                    shoot();
                    pathState = PathState.prepPickup2;
                }

            case prepPickup2:
                if (!follower.isBusy()) {

                    intakeMotor.setPower(1);

                    follower.followPath(toPrepP2, true);
                    pathState = PathState.pickup2;

                }

            case pickup2:
                if (!follower.isBusy()) {
                    follower.followPath(toP2, true);
                    pathState = PathState.returnShootPos2;

                }

            case returnShootPos2:
                if (!follower.isBusy()) {

                    intakeMotor.setPower(0);

                    follower.followPath(returnShootPos2, true);
                    pathState = PathState.shootR2;

                }

            case shootR2:
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


    public void shoot(){


        for (int i = 0; i <=3; i++) {
            shootBlock.setPosition(0);


            intakeMotor.setPower(1);
            sleep(300);
            intakeMotor.setPower(0);


            if(i !=3) {
                sleep(500);
            }




        }


        shootBlock.setPosition(0.5);


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
            shooter1.setVelocity(1650);
            shooter2.setVelocity(1650);




        }


    }
}

