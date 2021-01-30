package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.OdometryGlobalCoordinatePosition;

public class StudBot {





    //public CRServo servoLoop = null;
    //private float armAngle = 0;
    //public DcMotor arm = null;

    //public DigitalChannel digIn1;

    //public float arm_zero_position=0;
    double vertical,horizontal,pivot = 0.0;



    // odometry stuff

    DcMotor verticalRight, verticalLeft, horizontalRight;

    //The amount of encoder ticks for each inch the robot moves. This will change for each robot and needs to be changed here
    //final double COUNTS_PER_INCH = 307.699557;
    //final double COUNTS_PER_INCH = 307.699557/0.71803066;

    //Hardware map names for the encoder wheels. Again, these will change for each robot and need to be updated below
    //String verticalLeftEncoderName = "rf", verticalRightEncoderName = "lf", horizontalEncoderName = "lb";
    String rfName = "right_front_drive", rbName = "right_back_drive", lfName = "left_front_drive", lbName = "left_back_drive";
    String verticalLeftEncoderName = rbName, verticalRightEncoderName = lfName, horizontalEncoderName = rfName, horizontalRightEncoderName = lbName;

    //String rfName = "right_front_drive", rbName = "right_back_drive", lfName = "left_front_drive", lbName = "left_back_drive";


    public OdometryGlobalCoordinatePosition globalPositionUpdate;






    StudShooter shooter = new StudShooter();
    StudDrive drive = new StudDrive();
    StudIntake intake = new StudIntake();
    StudElevator elevator = new StudElevator();
    StudArm arm = new StudArm();
    StudIMU imu = new StudIMU();



    public void init(HardwareMap hardwareMap) {

        StudShooter shooter = new StudShooter();
        StudDrive drive = new StudDrive();
        StudIntake intake = new StudIntake();
        StudElevator elevator = new StudElevator();
        StudArm arm = new StudArm();
        StudIMU imu = new StudIMU();
        drive.init(hardwareMap);
        shooter.init(hardwareMap);
        intake.init(hardwareMap);
        elevator.init(hardwareMap);
        arm.init(hardwareMap);
        imu.init(hardwareMap);
/*
        digIn1 = hardwareMap.get(DigitalChannel.class, "switch1"); //false is not pressed
        digIn1.setMode(DigitalChannel.Mode.INPUT);

        arm  = hardwareMap.get(DcMotor.class, "arm");
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        servoLoop  = hardwareMap.crservo.get("servo_loop");
*/

        // odometry init
        //Assign the hardware map to the odometry wheels
        verticalLeft = hardwareMap.dcMotor.get(verticalLeftEncoderName);
        verticalRight = hardwareMap.dcMotor.get(verticalRightEncoderName);
        horizontalRight = hardwareMap.dcMotor.get(horizontalEncoderName);

        //Reset the encoders
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*
        Reverse the direction of the odometry wheels. THIS WILL CHANGE FOR EACH ROBOT. Adjust the direction (as needed) of each encoder wheel
        such that when the verticalLeft and verticalRight encoders spin forward, they return positive values, and when the
        horizontal encoder travels to the right, it returns positive value
        */
        //verticalLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //verticalRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //horizontalRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //Set the mode of the odometry encoders to RUN_WITHOUT_ENCODER
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft,
                verticalRight, horizontalRight, getDrive().COUNTS_PER_INCH, 75);

        Thread positionThread = new Thread(globalPositionUpdate);
        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseNormalEncoder();
        positionThread.start();



    }

    public void my_sleep(float ms){
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }


/*
    public void init_arm(){
        //going down

        arm.setPower(0.2);
        my_sleep(300);


        //going up
        arm.setPower(-0.2);
        do{
            my_sleep(10);
        }while(!digIn1.getState());
        arm.setPower(0);

        arm_zero_position=arm.getCurrentPosition();
    }
*/
    public StudShooter getShooter() {
        return shooter;
    }

    public StudDrive getDrive() {
        return drive;
    }

    public StudIntake getIntake() {
        return intake;
    }

    public StudElevator getElevator() {
        return elevator;
    }
    public StudArm getArm() {
        return arm;
    }
    public StudIMU getIMU() {
        return imu;
    }


    public void simpleMove (double desiredX, double desiredY, double desiredHeading){
        getDrive().fastPID();
        getDrive().setTargetfromOrgin(desiredX, desiredY, desiredHeading);

        double myDistance=1e10;

        while (myDistance>4) {


            myDistance= getDrive().move(globalPositionUpdate.returnXCoordinateInInches(),
                    globalPositionUpdate.returnYCoordinateInInches(),
                    getIMU().getZAngle() + getDrive().headingOffset);
        }
        getDrive().stopRobot();
    }


    public void accurateMove (double desiredX, double desiredY, double desiredHeading){
        getDrive().accuratePIDMove();
        getDrive().setTargetfromOrgin(desiredX, desiredY, desiredHeading);

        double myDistance=1e10;

        while (myDistance>4) {


            myDistance= getDrive().move(globalPositionUpdate.returnXCoordinateInInches(),
                    globalPositionUpdate.returnYCoordinateInInches(),
                    getIMU().getZAngle() + getDrive().headingOffset);
        }
        getDrive().stopRobot();
    }

    public void simpleTankMove (double in, double speed) {
        double myDistance = 0;
        double startx = globalPositionUpdate.returnXCoordinateInInches();
        double starty = globalPositionUpdate.returnYCoordinateInInches();
        double currentx = globalPositionUpdate.returnXCoordinateInInches();
        double currenty = globalPositionUpdate.returnYCoordinateInInches();
        while (myDistance<in) {

            currentx = globalPositionUpdate.returnXCoordinateInInches();
            currenty = globalPositionUpdate.returnYCoordinateInInches();
            getDrive().tankMove (speed);
            myDistance = Math.sqrt(
                    (startx-currentx)*(startx-currentx) +  (starty-currenty)*(starty-currenty));

        }
        getDrive().stopRobot();
    }


    public void simplePivot (double desiredHeading){

        getDrive().fastPID();
        getDrive().setTargetfromOrginHeading(desiredHeading);

        double myDistance=1e10;

        while (myDistance>4) {


            myDistance= getDrive().pivot(
                    getIMU().getZAngle() + getDrive().headingOffset);
        }
        getDrive().stopRobot();
    }




    public void accuratePivot (double desiredHeading){
        getDrive().accuratePID();
        getDrive().setTargetfromOrginHeading(desiredHeading);

        getDrive().iteration=0;

        while (getDrive().iteration<200) {


            getDrive().pivot(
                    getIMU().getZAngle() + getDrive().headingOffset);
        }
        getDrive().stopRobot();
    }

    public void moveToShoot(double azum) {
        //going up
        getElevator().elevator.setPower(0.8);
        while (getElevator().elevator.getCurrentPosition() - getElevator().elevator_zero_position < azum) {

            if (getElevator().readyToShoot()) {
                getShooter().servoBlock.setPosition(0.3);
            } else {
                //shooterUpdate.setBackMode();
                getShooter().servoBlock.setPosition(-0.5);
            }

            if (getElevator().elevator.getCurrentPosition() - getElevator().elevator_zero_position > azum - 500) {
                getElevator().elevator.setPower(0.4);
            }

            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        getElevator().elevator.setPower(0);

    }

    public void moveToPickup(){
            //going up
            getElevator().elevator.setPower(-0.8);
            while (getElevator().elevator.getCurrentPosition() - getElevator().elevator_zero_position > 50) {

                if (getElevator().readyToShoot()) {
                    getShooter().servoBlock.setPosition(0.3);
                } else {
                    //shooterUpdate.setBackMode();
                    getShooter().servoBlock.setPosition(-0.5);
                }

                if (getElevator().elevator.getCurrentPosition() - getElevator().elevator_zero_position > 500){
                    getElevator().elevator.setPower(-0.4);
                }

                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }


        getElevator().elevator.setPower(0);

    }



}
