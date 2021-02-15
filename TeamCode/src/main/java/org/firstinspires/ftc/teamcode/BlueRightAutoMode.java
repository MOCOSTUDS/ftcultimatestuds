package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "Blue Right Auto")
public class BlueRightAutoMode extends LinearOpMode {

    StudBot studbot = new StudBot();


    @Override
    public void runOpMode() throws InterruptedException {

        studbot.init(hardwareMap);
        studbot.getElevator().moveToShoot(3670);
        waitForStart();
        ShooterBeltSpeedVelocity shooterUpdate = new ShooterBeltSpeedVelocity(studbot.getShooter(), 100);
        Thread shooterThread = new Thread(shooterUpdate);
        shooterThread.start();

        studbot.getDrive().bornIn(48, 0, -90);


        //shooterUpdate.maintainSpeedModeStart(0.9);
        studbot.getIntake().stopAll();


        while (opModeIsActive()) {
            shooterUpdate.maintainSpeedModeStart(1.8);
            studbot.simpleMove(48, 55, -90.0);
            studbot.pivotAndElevate(-5.8,3700);
            //ring 1 & 2
            int i;
            for (i = 0; i < 2; i++) {
                studbot.getShooter().setClawShoot();
                Thread.sleep(800);
                studbot.getShooter().setClawOpen();
                Thread.sleep(800);
                studbot.getIntake().setFeed();
            }
            //ring 3
            studbot.pivotAndElevate(-5.8,3800);
            studbot.getShooter().setClawShoot();
            Thread.sleep(1400);
            studbot.getShooter().setClawOpen();
            studbot.getIntake().setBack();
            //studbot.accuratePivot(0);
            studbot.pivotAndElevateFast(0,3700);
            studbot.simpleTankMove(38,0.8,0);
            //studbot.simpleMove(48, 100, -90.0);
            //studbot.simpleMove(38, 100, -90.0);
            studbot.pivotAndElevateFast(-90,3700);
            //studbot.accuratePivot(-90);
            studbot.simpleTankMove(19,0.8,-90);
            studbot.accuratePivot(-135);
            //studbot.simpleMove(28, 100, -90.0);
            //studbot.simpleMove(0, 100, -90.0);

            studbot.arm.dropBobber();
            studbot.getArm().moveUp();
            shooterUpdate.maintainSpeedModeStart(1.8);
            studbot.pivotAndElevateFast(-90,3700);
            //studbot.accuratePivot(-90);
            studbot.simpleTankMove(45,-0.8,-90);
            //studbot.simpleMove(48, 100, -90); //previous y was 60
            //studbot.simpleMove(48, 75, -90); //previous y was 60
            studbot.simpleTankMoveHoriz(35,0.8,-90);
            studbot.accuratePivot(0);

            studbot.getElevator().moveToPickup();
            studbot.getArm().openLoop();

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
