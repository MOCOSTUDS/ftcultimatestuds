package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "Blue Teleop")
public class BlueLeftTeleop extends LinearOpMode {

    StudBot studbot = new StudBot();
    boolean was_pressed=false;


    @Override
    public void runOpMode() throws InterruptedException {

        studbot.init(hardwareMap);
        waitForStart();
        ShooterBeltSpeedVelocity shooterUpdate = new ShooterBeltSpeedVelocity(studbot.getShooter(), 100);
        Thread shooterThread = new Thread(shooterUpdate);

        shooterThread.start();


        /**
         * *****************
         * OpMode Begins Here
         * *****************
         */
        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions\
        /*
        OdometryGlobalCoordinatePosition globalPositionUpdate =
                new OdometryGlobalCoordinatePosition(studbot.verticalLeft,
                        studbot.verticalRight, studbot.horizontalRight, studbot.getDrive().COUNTS_PER_INCH, 75);



        Thread positionThread = new Thread(globalPositionUpdate);
        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseNormalEncoder();
        positionThread.start();
*/
        studbot.getDrive().bornIn(10, 0, 0);


        shooterUpdate.maintainSpeedModeStart(1.8);
        //studbot.getIntake().stopAll();
        //studbot.getElevator().moveToShoot(3600);
        //shooterUpdate.stop();
        while (opModeIsActive()) {


            /***************************************************************************************************
             * gamepad 2
             ****************************************************************************************************/

            if (gamepad2.right_trigger > 0.2) {
                studbot.getShooter().setClawShoot();
            } else {
                studbot.getShooter().setClawOpen();
            }
            studbot.getElevator().moveElevator(-(gamepad2.right_stick_y * gamepad2.right_stick_y * gamepad2.right_stick_y));
            // handle elevator
            if (studbot.getElevator().readyToShoot()) {
                shooterUpdate.maintainSpeedModeStart(2.0);
                studbot.getElevator().servoBlock.setPosition(0.3);
            } else {
                //shooterUpdate.setBackMode();
                shooterUpdate.maintainSpeedModeStart(2.0);
                studbot.getElevator().servoBlock.setPosition(-0.5);

            }


            if (gamepad2.left_bumper && !was_pressed) {
                studbot.getDrive().heading_correction -= 4;
                studbot.pivotAndElevate(0,3756);
                was_pressed = true;
            }
            if (gamepad2.right_bumper && !was_pressed) {
                studbot.getDrive().heading_correction += 4;
                studbot.pivotAndElevate(0,3756);
                was_pressed = true;
            }
            if (!gamepad2.left_bumper && !gamepad2.right_bumper)
                was_pressed = false;

            if (gamepad2.dpad_left && gamepad2.x) {
                studbot.pivotAndElevate(35,3774);
            }

            if (gamepad2.dpad_up && gamepad2.x) {
                studbot.pivotAndElevate(10,3772);
            }

            if (gamepad2.dpad_right && gamepad2.x) {
                studbot.pivotAndElevate(-7.5,3800);
            }

            if (gamepad2.dpad_left && gamepad2.y) {
                studbot.pivotAndElevate(25,3680);
            }

            if (gamepad2.dpad_up && gamepad2.y) {
                studbot.pivotAndElevate(12,3680);
            }

            if (gamepad2.dpad_right && gamepad2.y) {
                studbot.pivotAndElevate(-1.7,3612);
            }

            if (gamepad2.dpad_left && gamepad2.b) {
                studbot.pivotAndElevate(25,3400);
            }
            if (gamepad2.dpad_up && gamepad2.b) {
                studbot.pivotAndElevate(12,3400);
            }

            if (gamepad2.dpad_right && gamepad2.b) {
                studbot.pivotAndElevate(14,3370);
            }

            if (gamepad2.dpad_down && gamepad2.b) {
                studbot.pivotAndElevate(27,3330);
            }

            if (gamepad2.start) {
                studbot.pivotAndElevate(10,3462);
            }

            /***************************************************************************************************
             * gamepad 1
             ****************************************************************************************************/

            if (gamepad1.x || gamepad2.a) {
                studbot.getIntake().setBack();
            } else {
                studbot.getIntake().setFeed();
            }

            if (gamepad1.left_trigger>0.1){
                studbot.getArm().arm.setPower(0.6);
            }else
                if (gamepad1.left_bumper){
                    studbot.getArm().arm.setPower(-0.6);
                }else{
                    studbot.getArm().arm.setPower(0);
                }


            if (gamepad1.right_trigger>0.1){
                studbot.getArm().servoLoop.setPower(-1);
            }else
                if (gamepad1.right_bumper){
                    studbot.getArm().servoLoop.setPower(1);
                }else{
                    studbot.getArm().servoLoop.setPower(0);
                }

            double vertical = -1.0 * gamepad1.left_stick_x;
            double horizontal = gamepad1.left_stick_y;
            double pivot = gamepad1.right_stick_x;
            double h = -studbot.getIMU().getZAngle();
            double newHorizontal = horizontal *Math.cos(Math.toRadians(h))
                    - vertical*Math.sin(Math.toRadians(h));

            double newVertical = horizontal*Math.sin(Math.toRadians(h))
                    +vertical*Math.cos(Math.toRadians(h)) ;

            double newPivot = pivot ;
            studbot.getDrive().teleDrive(newVertical, newHorizontal, newPivot);


            /*
            if (gamepad2.start) {
                studbot.arm.dropBobber();
            }
*/
            telemetry.addData("imu offset", studbot.getDrive().heading_correction);

            telemetry.addData("Velocity", shooterUpdate.returnVelocity());
            //telemetry.addData("Power", shooterUpdate.returnPower());
            //telemetry.addData("Thread Active", shooterThread.isAlive());
            //telemetry.addData("switch0", studbot.getElevator().digIn0.getState());
            //telemetry.addData("switch1=", studbot.getArm().digIn1.getState());
            //telemetry.addData("detaltT=", shooterUpdate.getLastTime());
            telemetry.addData("elevator=", studbot.getElevator().getElevatorPosition() - studbot.getElevator().elevator_zero_position);

            telemetry.addData("X Position", studbot.globalPositionUpdate.returnXCoordinateInInches());
            telemetry.addData("Y Position", studbot.globalPositionUpdate.returnYCoordinateInInches());
            telemetry.addData("Orientation (Degrees)", studbot.globalPositionUpdate.returnOrientation());
            //telemetry.addData("Thread Active", positionThread.isAlive());

            //telemetry.addData("vertical left", studbot.verticalLeft.getCurrentPosition());
            //telemetry.addData("vertical right", studbot.verticalRight.getCurrentPosition());
            //telemetry.addData("horizontal", studbot.horizontalRight.getCurrentPosition());

            telemetry.addData("armPostion", studbot.getArm().getarmPosition());
            telemetry.addData("imuAngle", studbot.getIMU().getZAngle());

            //telemetry.addData("servoBlock:", studbot.getShooter().servoBlock);

            //telemetry.update();
           // telemetry.addData("iteration: ", studbot.getDrive().iteration);
            //telemetry.addData("ready to shoot",studbot.getElevator().readyToShoot());
            telemetry.update();

        }
        //Stop the thread
        shooterUpdate.stop();
        studbot.getIntake().stopAll();


    }
}
