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


@Autonomous (name="hpSpamRedFar", group="Linear OpMode")
public class hpSpamRedFar extends  LinearOpMode {

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

        cycle,

        toEndPose
    }

    PathState pathState;


    //all the poses the robot will be in when something happens
    private final Pose startPos = new Pose(84, 8, Math.toRadians(270));
    private final Pose shootPos = new Pose(84, 18, Math.toRadians(256));

    private final Pose prepPickup1 = new Pose(96, 35, Math.toRadians(0));
    private final Pose pickup1 = new Pose(142, 35, Math.toRadians(0));

    private final Pose pickupHP = new Pose(142, 12, Math.toRadians(90));

    private final Pose endPose = new Pose(112, 12, Math.toRadians(90));


    //these are the paths the robot will follow, one pose to another
    private PathChain startPosToShootPos, shootPosToPrepP1, prepP1ToP1, returnShootPos1, shootToHP, hpToShoot, toEndPos;

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

        shootToHP = follower.pathBuilder()
                .addPath(new BezierLine(shootPos, pickupHP))
                .setLinearHeadingInterpolation(shootPos.getHeading(), pickupHP.getHeading())
                .build();

        hpToShoot = follower.pathBuilder()
                .addPath(new BezierLine(pickupHP, shootPos))
                .setLinearHeadingInterpolation(pickupHP.getHeading(), shootPos.getHeading())
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
                    pathState = PathState.cycle;
                }


            case cycle:
                if (!follower.isBusy()){

                    for (int i = 0; i <=3; i++) {

                        intakeMotor.setPower(1);
                        follower.followPath(shootToHP);

                        if(!follower.isBusy()) {
                            sleep(2000);
                            intakeMotor.setPower(0);

                            follower.followPath(hpToShoot);
                        }

                        if(!follower.isBusy()) {
                            shoot();
                        }
                    }

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
            sleep(500);


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

            telemetry.addData("path state ", pathState.toString());

            shootBlock.setPosition(0.5);
            pivot.setPosition(1);
            shooter1.setVelocity(1975);
            shooter2.setVelocity(1975);


        }

    }
}
