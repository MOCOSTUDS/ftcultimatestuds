package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.OdometryGlobalCoordinatePosition;


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
        studbot.getDrive().bornIn(10, 0, -90);


        //shooterUpdate.maintainSpeedModeStart(0.9);
        studbot.getIntake().stopAll();
        while (opModeIsActive()) {
            if (gamepad2.right_trigger > 0.2) {
                studbot.getShooter().setClawShoot();
            } else {
                studbot.getShooter().setClawOpen();
            }

            /*
            if (gamepad2.left_bumper) {
                studbot.getIntake().setBack();
            } else {
                studbot.getIntake().setFeed();
            }
            */

            studbot.getElevator().moveElevator(-(gamepad2.right_stick_y * gamepad2.right_stick_y * gamepad2.right_stick_y));
            // handle elevator
            if (studbot.getElevator().readyToShoot()) {
                shooterUpdate.maintainSpeedModeStart(2.0);
                studbot.getShooter().servoBlock.setPosition(0.3);
            } else {
                //shooterUpdate.setBackMode();
                shooterUpdate.maintainSpeedModeStart(2.0);
                studbot.getShooter().servoBlock.setPosition(-0.5);

            }

            if (gamepad1.back){
                studbot.accuratePivot(10);
                studbot.moveToShoot(3750);
                int i;
                for (i = 0; i<4; i++) {
                    studbot.getShooter().setClawShoot();
                    Thread.sleep(1000);
                    studbot.getShooter().setClawOpen();
                    Thread.sleep(1000);
                }
            }else{
                //studbot.getShooter().servoBlock.setPosition(-0.5);
            }



            if (gamepad1.y) {
                studbot.getDrive().setTarget(0.0, 40.0, 0.0);
                studbot.getDrive().move(studbot.globalPositionUpdate.returnXCoordinateInInches(),
                        studbot.globalPositionUpdate.returnYCoordinateInInches(),
                        studbot.globalPositionUpdate.returnOrientation());


            } else if (gamepad1.x) {
                studbot.getDrive().setTarget(40.0, 0.0, 00.0);
                studbot.getDrive().move(studbot.globalPositionUpdate.returnXCoordinateInInches(),
                        studbot.globalPositionUpdate.returnYCoordinateInInches(),
                        studbot.globalPositionUpdate.returnOrientation());


            } else if (gamepad1.a) {
                studbot.getDrive().setTarget(0.0, 0.0, 0.0);
                studbot.getDrive().move(studbot.globalPositionUpdate.returnXCoordinateInInches(),
                        studbot.globalPositionUpdate.returnYCoordinateInInches(),
                        studbot.globalPositionUpdate.returnOrientation());


            } else if (gamepad2.dpad_down) {
                studbot.getDrive().setTarget(0.0, 0.0, 0.0);
                studbot.getDrive().move(studbot.globalPositionUpdate.returnXCoordinateInInches(),
                        studbot.globalPositionUpdate.returnYCoordinateInInches(),
                        studbot.globalPositionUpdate.returnOrientation());
            } else if (gamepad2.dpad_left) {
                studbot.getDrive().setTarget(0.0, 0.0, -90.0);
                studbot.getDrive().move(studbot.globalPositionUpdate.returnXCoordinateInInches(),
                        studbot.globalPositionUpdate.returnYCoordinateInInches(),
                        studbot.globalPositionUpdate.returnOrientation());
            } else if (gamepad2.dpad_right) {
                studbot.getDrive().setTarget(0.0, 0.0, 90.0);
                studbot.getDrive().move(studbot.globalPositionUpdate.returnXCoordinateInInches(),
                        studbot.globalPositionUpdate.returnYCoordinateInInches(),
                        studbot.globalPositionUpdate.returnOrientation());
            } else if (gamepad2.dpad_up) {
                studbot.getDrive().setTarget(0.0, 0.0, 180.0);
                studbot.getDrive().move(studbot.globalPositionUpdate.returnXCoordinateInInches(),
                        studbot.globalPositionUpdate.returnYCoordinateInInches(),
                        studbot.globalPositionUpdate.returnOrientation());
            } else if (gamepad1.b) {
                studbot.getDrive().setTarget(0.0, 0.0, -90.0);
                studbot.getDrive().move(studbot.globalPositionUpdate.returnXCoordinateInInches(),
                        studbot.globalPositionUpdate.returnYCoordinateInInches(),
                        studbot.globalPositionUpdate.returnOrientation());
            } else if (gamepad2.x) {


                studbot.simpleMove(10, 107, -90.0);

                studbot.arm.dropBobber();

                studbot.simpleMove(35,65,-90);

                studbot.accuratePivot(8);
                studbot.moveToShoot(3730);
                int i;
                for (i = 0; i<4; i++) {
                    studbot.getShooter().setClawShoot();
                    Thread.sleep(1000);
                    studbot.getShooter().setClawOpen();
                    Thread.sleep(1000);
                    studbot.getIntake().setFeed();
                }
                //studbot.simplePivot(45);
                //studbot.simplePivot(-90);
                studbot.accuratePivot(-140);
                studbot.moveToPickup();
                studbot.simpleTankMove(6,1);
                studbot.getIntake().setFeed();
                studbot.simpleTankMove(25,0.3);
                studbot.simpleTankMove(31,-0.3);
                studbot.moveToShoot(3730);
                studbot.accuratePivot(0);
                for (i = 0; i<4; i++) {
                    studbot.getShooter().setClawShoot();
                    Thread.sleep(1000);
                    studbot.getShooter().setClawOpen();
                    Thread.sleep(1000);
                    studbot.getIntake().setFeed();
                }
                studbot.simpleTankMove(15,1);

                //studbot.simplePivot(85);

                //studbot.simpleMove(0,12,-90);

            } else {

                // do drive  old way
                /*
                double vertical = -1.0 * gamepad1.left_stick_y;
                double horizontal = -gamepad1.left_stick_x;
                double pivot = gamepad1.right_stick_x;
                studbot.getDrive().teleDrive(vertical, horizontal, pivot);
                */

                //Math for field orientated drive
                /*
                x*Math.cos(Math.toRadians(headingOffset))
                        -y*Math.sin(Math.toRadians(headingOffset));
                x*Math.sin(Math.toRadians(headingOffset))
                        +y*Math.cos(Math.toRadians(headingOffset));
                */

                double vertical = -1.0 * gamepad1.left_stick_y;
                double horizontal = -gamepad1.left_stick_x;
                double pivot = gamepad1.right_stick_x;

    double h = -studbot.getIMU().getZAngle();
                double newHorizontal = horizontal *Math.cos(Math.toRadians(h))
                        - vertical*Math.sin(Math.toRadians(h));

                double newVertical = horizontal*Math.sin(Math.toRadians(h))
                        +vertical*Math.cos(Math.toRadians(h)) ;

                double newPivot = pivot ;
                studbot.getDrive().teleDrive(newVertical, newHorizontal, newPivot);


            }
            if (gamepad2.start) {
                studbot.arm.dropBobber();
            }



            telemetry.addData("Velocity", shooterUpdate.returnVelocity());
            //telemetry.addData("Power", shooterUpdate.returnPower());
            //telemetry.addData("Thread Active", shooterThread.isAlive());
            //telemetry.addData("switch0", studbot.getElevator().digIn0.getState());
            telemetry.addData("switch1=", studbot.getArm().digIn1.getState());
            //telemetry.addData("detaltT=", shooterUpdate.getLastTime());
            telemetry.addData("elevator=", studbot.getElevator().getElevatorPosition() - studbot.getElevator().elevator_zero_position);

            //telemetry.addData("X Position", studbot.globalPositionUpdate.returnXCoordinateInInches());
            //telemetry.addData("Y Position", studbot.globalPositionUpdate.returnYCoordinateInInches());
            telemetry.addData("Orientation (Degrees)", studbot.globalPositionUpdate.returnOrientation());
            //telemetry.addData("Thread Active", positionThread.isAlive());

            //telemetry.addData("vertical left", studbot.verticalLeft.getCurrentPosition());
            //telemetry.addData("vertical right", studbot.verticalRight.getCurrentPosition());
            //telemetry.addData("horizontal", studbot.horizontalRight.getCurrentPosition());

            telemetry.addData("armPostion", studbot.getArm().getarmPosition());
            telemetry.addData("imuAngle", studbot.getIMU().getZAngle());

            //telemetry.addData("servoBlock:", studbot.getShooter().servoBlock);

            //telemetry.update();
            telemetry.addData("iteration: ", studbot.getDrive().iteration);
            telemetry.addData("ready to shoot",studbot.getElevator().readyToShoot());
            telemetry.update();

        }
        //Stop the thread
        shooterUpdate.stop();
        studbot.getIntake().stopAll();


    }
}
