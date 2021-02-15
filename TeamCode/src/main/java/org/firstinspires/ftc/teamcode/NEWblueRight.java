package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "New Blue Right Auto")
public class NEWblueRight extends LinearOpMode {

    StudBot studbot = new StudBot();

    ShooterBeltSpeedVelocity shooterUpdate;

    ElevatorPosition elevatorUpdate;

    void mySleep(int mil) {
         try {
        Thread.sleep( mil);
        } catch(Exception e)
         { }
    }
    public void doZoneA() {
        studbot.simpleMove(48, 55, -90.0);
        studbot.getElevator().setTarget(3700);
        studbot.accuratePivot(-2.4);


        //ring 1 & 2
        int i;
        for (i = 0; i < 2; i++) {
            studbot.getShooter().setClawShoot();
            mySleep(800);
            studbot.getShooter().setClawOpen();
            mySleep(800);
            //if (i==1)
            studbot.getIntake().setFeed();
        }
        //ring 3m
        studbot.accuratePivot(0);
        studbot.getElevator().setTarget(3800);
        mySleep(1300);
        studbot.getShooter().setClawShoot();
        mySleep(1400);
        studbot.getShooter().setClawOpen();


        studbot.getIntake().setBack();
        studbot.getElevator().setTarget(3700);
        studbot.accuratePivot(0);

        studbot.simpleTankMove(2,0.8,0);

        studbot.accuratePivot(-90);

        studbot.getElevator().setTarget(0);
        studbot.simpleTankMove(8,0.8,-90);
        studbot.accuratePivot(-160);

        studbot.arm.dropBobber();
        studbot.getArm().moveUp();
        shooterUpdate.maintainSpeedModeStart(1.8);
        studbot.accuratePivot(-90);
        studbot.simpleTankMoveHoriz(24,-0.8,-90);
        studbot.simpleTankMove(30,-0.8,-90);

        studbot.accuratePivot(0);

        studbot.simpleTankMove(10,-0.8,0);


        //studbot.simpleTankMoveHoriz(5,0.8,-90);
       // studbot.accuratePivot(0);

        //studbot.getElevator().moveToPickup();

        studbot.getArm().moveUp();

    };


    public void doZoneB() {
        studbot.simpleTankMoveHoriz(31,-0.8,-80);
        //studbot.simpleMove(48, 55, -90.0);
        studbot.getElevator().setTarget(3460);

        studbot.accuratePivot(0);

        //ring 1 & 2
        int i;
        for (i = 0; i < 3; i++) {
            studbot.getShooter().setClawShoot();
            mySleep(800);
            studbot.getShooter().setClawOpen();
            mySleep(1200);
            //if (i==1)
            studbot.getIntake().setFeed();
        }
        //ring 3
        studbot.getElevator().setTarget(3550);
        studbot.getShooter().setClawShoot();
        mySleep(1800);
        studbot.getShooter().setClawOpen();

        studbot.getElevator().setTarget(0);
        studbot.accuratePivot(-90);
        mySleep(800);
        studbot.simpleTankMove(13,0.6,-90);
        mySleep(3000);


        studbot.getElevator().setTarget(3900);
        studbot.simpleTankMove(13,-0.6,-90);
        studbot.accuratePivot(5);
        mySleep(2000);

        //ring 4

        studbot.getShooter().setClawShoot();
        mySleep(800);
        studbot.getElevator().setTarget(150);
        studbot.accuratePivot(-90);

        studbot.simpleTankMoveHoriz(41,-0.8,-100);
        studbot.getShooter().setClawOpen();
        studbot.getArm().dropBobber();
        studbot.simpleTankMoveHoriz(4,0.8,-100);
        studbot.accuratePivot(0);
        studbot.getArm().moveUp();

        //studbot.getArm().openLoop();

    }

    public void doZoneC() {
        studbot.simpleTankMoveHoriz(2,-0.8,-90);
        studbot.getElevator().setTarget(3700);
        studbot.simpleTankMove(10,0.8,-90);
        studbot.accurateFastPivot(13);

        //*******ring 1*********
        studbot.simpleTankMove(6,0.5,13);
        studbot.getShooter().setClawShoot();
        studbot.simpleTankMove(6,0.5,13);
        studbot.getShooter().setClawOpen();
        studbot.getIntake().setFeed();
        studbot.getIntake().setFrontIntakeBack();
        studbot.getElevator().setTarget(3750);
        //*******ring 2**********
        studbot.simpleTankMove(6,0.5,13);
        studbot.getShooter().setClawShoot();
        studbot.simpleTankMove(6,0.5,13);
        studbot.getElevator().setTarget(3570);
        studbot.getShooter().setClawOpen();
        //studbot.getElevator().setTarget(3650);
        //*******ring 3*********
        //studbot.simpleTankMove(5,0.2,5);
        mySleep(1000);
        studbot.getShooter().setClawShoot();
        //studbot.simpleTankMove(5,0.2,5);
        mySleep(600);
        studbot.getShooter().setClawOpen();
        mySleep(1000);
        studbot.getShooter().setClawShoot();
        //studbot.simpleTankMove(5,0.2,5);
        mySleep(600);
        studbot.getShooter().setClawOpen();
        studbot.getElevator().setTarget(0);
        mySleep(1800);
        studbot.getIntake().setFeed();

            // first 2 rings


        int i;
        for (i =0; i< 2; i++) {
            studbot.getIntake().setFeed();
            studbot.simpleTankMove(7, 0.3, 13);
            studbot.getIntake().setBeltIntakeBack();
            studbot.simpleTankMove(2, -0.4, 13);
        }
        studbot.getIntake().setShooterIntakesBack();
        mySleep(1000);
        studbot.getElevator().setTarget(3870);

        mySleep(200);
        studbot.getIntake().setBack();
        mySleep(2000);
        studbot.getIntake().setFeed();
        //studbot.simpleTankMove(40, 0.3, 13);
        for (i = 0; i < 2; i++) {
            studbot.getShooter().setClawShoot();
            mySleep(800);
            studbot.getShooter().setClawOpen();
            mySleep(1200);
            //if (i==1)
            studbot.getIntake().setFeed();
        }

/*
        // second 2 rings

        studbot.getElevator().setTarget(0);
        mySleep(1800);
        studbot.getIntake().setFeed();

        for (i =0; i< 2; i++) {
            studbot.getIntake().setFeed();
            studbot.simpleTankMove(7, 0.3, 13);
            studbot.getIntake().setBeltIntakeBack();
            studbot.simpleTankMove(2, -0.4, 13);
        }
        studbot.getIntake().setShooterIntakesBack();
        mySleep(1000);
        studbot.getElevator().setTarget(3870);
        mySleep(500);
        studbot.getIntake().setBack();
        mySleep(1000);
        studbot.getIntake().setFeed();
        //studbot.simpleTankMove(40, 0.3, 13);
        for (i = 0; i < 2; i++) {
            studbot.getShooter().setClawShoot();
            mySleep(800);
            studbot.getShooter().setClawOpen();
            mySleep(1200);
            //if (i==1)
            studbot.getIntake().setFeed();
        }
      */
        studbot.getElevator().setTarget(150);

        studbot.accurateFastPivot(-90);
        studbot.simpleTankMoveHoriz(62,-0.8,-90);
        studbot.simpleTankMove(10,0.8,-90);
        studbot.getArm().dropBobber();
        studbot.simpleTankMove(30,-0.8,-90);
        studbot.simpleTankMoveHoriz(28,0.8,-90);
        studbot.accuratePivot(0);
        studbot.getArm().moveUp();


    }

    @Override
    public void runOpMode() throws InterruptedException {

        studbot.init(hardwareMap);
        studbot.getElevator().moveToShoot(3670);
        waitForStart();
        shooterUpdate = new ShooterBeltSpeedVelocity(studbot.getShooter(), 100);
        elevatorUpdate = new ElevatorPosition(studbot.getElevator(), 100);
        Thread shooterThread = new Thread(shooterUpdate);
        Thread elevatorThread = new Thread(elevatorUpdate);
        shooterThread.start();
        elevatorThread.start();

        studbot.getDrive().bornIn(48, 0, -90);


        //shooterUpdate.maintainSpeedModeStart(0.9);
        studbot.getIntake().stopAll();

        String targetZone = studbot.cam.determineZone();
        studbot.cam.stopAll();
        while (opModeIsActive()) {

            telemetry.addData("targetZone", targetZone);
            telemetry.addData("height",studbot.cam.ringHeight);
            telemetry.addData("objects", studbot.cam.numObjects);
            telemetry.update();
            studbot.getElevator().setTarget(3750);
            elevatorUpdate.setMovingMode();


            shooterUpdate.maintainSpeedModeStart(1.8);
            if (targetZone.equals("A")) {
                doZoneA();
                break;
            }
            if (targetZone.equals("B")) {
                doZoneB();
                break;
            }
            if (targetZone.equals("C")) {
                doZoneC();
                break;
            }


         /*

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
            break;*/
        }
        //Stop the thread
        shooterUpdate.stop();
        elevatorUpdate.stop();
        studbot.getIntake().stopAll();


    }
}
