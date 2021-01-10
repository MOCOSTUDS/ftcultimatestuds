package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Sarthak on 10/4/2019.
 */
@TeleOp(name = "Example OpMode")
public class ExampleOpmode extends LinearOpMode {

    StudBot studbot = new StudBot();

    @Override
    public void runOpMode() throws InterruptedException {

        studbot.init(hardwareMap);
        waitForStart();
        ShooterBeltSpeedVelocity shooterUpdate = new ShooterBeltSpeedVelocity(studbot.getShooter(), 75);
        Thread shooterThread = new Thread(shooterUpdate);
        studbot.getShooter().startShooter(30.0);
        shooterThread.start();

        while(opModeIsActive()) {
            telemetry.addData("Velocity", shooterUpdate.returnVelocity());
            telemetry.addData("Power", shooterUpdate.returnPower());
            telemetry.addData("Thread Active", shooterThread.isAlive());
            telemetry.update();
            if (gamepad1.a) {
                shooterUpdate.maintainSpeedModeStart(1.9);
            }
            if (gamepad1.b) {
                shooterUpdate.notShooting();
            }
        }
        //Stop the thread
        shooterUpdate.stop();

    }


}
