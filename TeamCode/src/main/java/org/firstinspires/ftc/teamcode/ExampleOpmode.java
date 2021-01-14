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

        while (opModeIsActive()) {
            telemetry.addData("Velocity", shooterUpdate.returnVelocity());
            telemetry.addData("Power", shooterUpdate.returnPower());
            telemetry.addData("Thread Active", shooterThread.isAlive());
            telemetry.addData("switch0", studbot.digIn0.getState());
            telemetry.addData("elevator=", studbot.elevator.getCurrentPosition() - studbot.elevator_zero_position);

            telemetry.update();

            if (studbot.elevator.getCurrentPosition() - studbot.elevator_zero_position >2000) {
                shooterUpdate.maintainSpeedModeStart(0.9);
            }else{
                shooterUpdate.notShooting(-0.3);

            }

            studbot.servoLoop.setPower(0);


            if (gamepad2.right_trigger > 0.2) {
                studbot.servoClaw.setPosition(1);
            } else {
                studbot.servoClaw.setPosition(0.3);
            }
            if (gamepad2.left_bumper){
                studbot.servoIntake.setPower(-1);
                studbot.intakeWheels.setPower(-0.5);
                studbot.intakeBelt.setPower(1);
            }else{
                studbot.servoIntake.setPower(1);
                studbot.intakeBelt.setPower(-1);
                studbot.intakeWheels.setPower(0.5);
            }
            if (gamepad2.b) {
                studbot.init_elevator();
            }
            if(gamepad1.dpad_down) studbot.arm.setPower(0.5);
            else if(gamepad1.dpad_up) studbot.arm.setPower(-0.5);
            else studbot.arm.setPower(0);

            if(gamepad1.dpad_right) studbot.servoLoop.setPower(1);
            else if(gamepad1.dpad_left) studbot.servoLoop.setPower(-1);
            else studbot.servoLoop.setPower(0);
            if(studbot.digIn0.getState()== true || (gamepad2.right_stick_y < 0.1 && gamepad2.right_stick_y > -0.1)) {
                studbot.elevator.setPower(0);
                
            }else{
                studbot.elevator.setPower(-(gamepad2.right_stick_y*gamepad2.right_stick_y*gamepad2.right_stick_y));
            }
            /*
            if(gamepad1.left_stick_y <0.1 && gamepad1.left_stick_y > -0.1) {
                studbot.arm.setPower(0);
            }else{
                studbot.arm.setPower(gamepad1.left_stick_y*gamepad1.left_stick_y*gamepad1.left_stick_y);
            }


            if(gamepad1.left_trigger >.1) {
                studbot.servoLoop.setPower(1);
            }else{
                if(gamepad1.left_bumper) {
                    studbot.servoLoop.setPower(-1);
                }else studbot.servoLoop.setPower(0);
            }

            */

        }
        //Stop the thread
        shooterUpdate.stop();

    }


}
