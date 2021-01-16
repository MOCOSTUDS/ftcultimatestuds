package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Sarthak on 10/4/2019.
 */
@TeleOp(name = "BuildUp OpMode")
public class BuildUpOpmode extends LinearOpMode {

    StudBot studbot = new StudBot();

    @Override
    public void runOpMode() throws InterruptedException {

        studbot.init(hardwareMap);
        waitForStart();
        ShooterBeltSpeedVelocity shooterUpdate = new ShooterBeltSpeedVelocity(studbot.getShooter(), 100);
        Thread shooterThread = new Thread(shooterUpdate);
        shooterThread.start();


        //shooterUpdate.maintainSpeedModeStart(0.9);
        while (opModeIsActive()) {
            if (gamepad2.right_trigger > 0.2) {
                studbot.getShooter().setClawShoot();
            } else {
                studbot.getShooter().setClawOpen();
            }
            if (gamepad2.left_bumper){
                studbot.getIntake().setBack();
            }else{
                studbot.getIntake().setFeed();
            }
            studbot.getElevator().moveElevator(-(gamepad2.right_stick_y*gamepad2.right_stick_y*gamepad2.right_stick_y));
            // handle elevator
            if (studbot.getElevator().readyToShoot())
            {
                shooterUpdate.maintainSpeedModeStart(2.0);
            } else {
                shooterUpdate.setBackMode();
            }
            // do drive
            double vertical= -1.0*gamepad1.left_stick_y;
            double horizontal = gamepad1.left_stick_x;
            double pivot = gamepad1.right_stick_x;
            studbot.getDrive().teleDrive(vertical,horizontal,pivot);


            telemetry.addData("Velocity", shooterUpdate.returnVelocity());
            telemetry.addData("Power", shooterUpdate.returnPower());
            telemetry.addData("Thread Active", shooterThread.isAlive());
            telemetry.addData("switch0", studbot.getElevator().digIn0.getState());
            telemetry.addData("switch1=", studbot.digIn1.getState());
            telemetry.addData("detaltT=", shooterUpdate.getLastTime());
            telemetry.addData("elevator=", studbot.getElevator().getElevatorPosition() - studbot.getElevator().elevator_zero_position);
            telemetry.update();

        }
        //Stop the thread
        shooterUpdate.stop();
        studbot.getIntake().stopAll();

    }


}
