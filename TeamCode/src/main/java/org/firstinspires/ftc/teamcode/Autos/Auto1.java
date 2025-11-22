package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.RoadrunnerFiles.MecanumDrive;

import java.util.Timer;
import java.util.TimerTask;


@Autonomous(name="auto1", group="Linear OpMode")
public class Auto1 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

    waitForStart();

    Timer timer = new Timer();
        TimerTask task = new TimerTask() {
            @Override
            public void run() {

            }
        };

    }
}