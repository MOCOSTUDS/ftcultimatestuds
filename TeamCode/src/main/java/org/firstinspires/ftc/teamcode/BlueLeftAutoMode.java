package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "Blue Left Auto")
public class BlueLeftAutoMode extends LinearOpMode {

    StudBot studbot = new StudBot();


    @Override
    public void runOpMode() throws InterruptedException {

        studbot.init(hardwareMap);
        studbot.getElevator().moveToShoot(3670);
        waitForStart();
        ShooterBeltSpeedVelocity shooterUpdate = new ShooterBeltSpeedVelocity(studbot.getShooter(), 100);
        Thread shooterThread = new Thread(shooterUpdate);
        shooterThread.start();

        studbot.getDrive().bornIn(10, 0, -90);


        //shooterUpdate.maintainSpeedModeStart(0.9);
        studbot.getIntake().stopAll();


        while (opModeIsActive()) {

            /*
            studbot.ppMove (6,107,-90);
            telemetry.addData("p1 = ", studbot.x1+","+studbot.y1);
            telemetry.addData("p2 = ", studbot.x2+","+studbot.y2);
            telemetry.addData("vector = ", studbot.vectorX+","+studbot.vectorY);
            telemetry.addData("rabbit = ", studbot.rabbitX+","+studbot.rabbitY);
            telemetry.addData("vert , hori, pivot = ", studbot.ppvertical+","+studbot.pphorizontal+","+studbot.pppivot);
            telemetry.addData("current x,y = ", studbot.currentX+","+studbot.currentY);
            telemetry.update();
            Thread.sleep(10000);

*/
            studbot.simpleMove(10, 10, -90.0);
            studbot.pivotAndElevate(0,3670);
            //studbot.simpleMove(10, 30, 0);



            studbot.simpleTankMove (82, 0.6, 0);

            //studbot.ppMove (6,107,-90);

            //Thread.sleep(5000);
            studbot.pivotAndElevateFast(-90,3670);
            /*
            studbot.simpleMove(15, 30, -90.0);
            studbot.simpleMove(15, 60, -90.0);
            studbot.simpleMove(10, 90, -90.0);
            studbot.simpleMove(6, 107, -90.0);
            */

            telemetry.addData("X Position", studbot.globalPositionUpdate.returnXCoordinateInInches());
            telemetry.addData("Y Position", studbot.globalPositionUpdate.returnYCoordinateInInches());
            studbot.getDrive().setTargetfromOrgin(studbot.globalPositionUpdate.returnXCoordinateInInches(),
                    studbot.globalPositionUpdate.returnYCoordinateInInches(), -90);
            telemetry.addData("target x", studbot.getDrive().targetX);
            telemetry.addData("target y", studbot.getDrive().targetY);
            telemetry.update();

            studbot.arm.dropBobber();
            studbot.getArm().moveUp();
            //Thread.sleep(800);
            shooterUpdate.maintainSpeedModeStart(1.8);
            studbot.pivotAndElevateFast(0,3670);
            studbot.simpleTankMove (43, -0.6, 0);
            //studbot.simpleMove(10, 63, 0); //previous y was 60

            //studbot.accuratePivot(20);
            studbot.pivotAndElevate(28,3670);
            //studbot.moveToShoot(3760); // previous was 3730

            //ring1
            studbot.getShooter().setClawShoot();
            Thread.sleep(800);
            studbot.getShooter().setClawOpen();
            Thread.sleep(800);
            studbot.getIntake().setFeed();

            //ring2
            studbot.getShooter().setClawShoot();
            Thread.sleep(800);
            studbot.getShooter().setClawOpen();
            Thread.sleep(1800);

            //ring3
            studbot.pivotAndElevate(28,3770);
            studbot.getShooter().setClawShoot();
            Thread.sleep(800);
            studbot.getShooter().setClawOpen();
            Thread.sleep(1800);


            studbot.simpleTankMove(10, 1);
            studbot.accuratePivot(0);
            //studbot.getArm().moveUp();
            studbot.getElevator().moveToPickup();

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
            break;
        }
        //Stop the thread
        shooterUpdate.stop();
        studbot.getIntake().stopAll();


    }
}
