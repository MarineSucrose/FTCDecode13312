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


@Autonomous (name="RedHP", group="Linear OpMode")
public class redHP extends  LinearOpMode {

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

        toHP1,
        returnShootPos1,
        shootR1,

        toHP2,
        returnShootPos2,
        shootR2,

        toHP3,
        returnShootPos3,
        shootR3,

        toEndPose
    }

    PathState pathState;


    //all the poses the robot will be in when something happens
    private final Pose startPos = new Pose(84, 8, Math.toRadians(270));
    private final Pose shootPos = new Pose(84, 18, Math.toRadians(246));
    private final Pose humanPlayer = new Pose(144, 10, Math.toRadians(0));
    private final Pose endPose = new Pose(112, 12, Math.toRadians(90));


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
                .setLinearHeadingInterpolation(humanPlayer.getHeading(), shootPos.getHeading())
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
                    pathState = PathState.toHP1;
                }




            case toHP1:
                if (!follower.isBusy()) {
                    intakeMotor.setPower(1);
                    follower.followPath(toHP, true);
                    pathState = PathState.returnShootPos1;
                }

            case returnShootPos1:
                if (!follower.isBusy()) {
                    sleep(1000);
                    intakeMotor.setPower(0);
                    follower.followPath(returnShootPos, true);
                    pathState = PathState.shootR1;
                }

            case shootR1:
                if (!follower.isBusy()) {
                    shoot();
                    pathState = PathState.toHP2;
                }





            case toHP2:
                if (!follower.isBusy()) {
                    intakeMotor.setPower(1);
                    follower.followPath(toHP, true);
                    pathState = PathState.returnShootPos2;
                }

            case returnShootPos2:
                if (!follower.isBusy()) {
                    sleep(1000);
                    intakeMotor.setPower(0);
                    follower.followPath(returnShootPos, true);
                    pathState = PathState.shootR2;
                }

            case shootR2:
                if (!follower.isBusy()) {
                    shoot();
                    pathState = PathState.toHP3;
                }




            case toHP3:
                if (!follower.isBusy()) {
                    intakeMotor.setPower(1);
                    follower.followPath(toHP, true);
                    pathState = PathState.returnShootPos3;
                }

            case returnShootPos3:
                if (!follower.isBusy()) {
                    sleep(1000);
                    intakeMotor.setPower(0);
                    follower.followPath(returnShootPos, true);
                    pathState = PathState.shootR3;
                }

            case shootR3:
                if (!follower.isBusy()) {
                    shoot();
                    pathState = PathState.toEndPose;
                }





            case toEndPose:
                if(!follower.isBusy()){
                    follower.followPath(toEndPos);
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
