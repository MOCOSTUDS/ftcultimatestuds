package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class StudBot {

    private DcMotor leftFrontDrive = null;
    private DcMotor leftRearDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightRearDrive = null;

    public CRServo intakeBelt = null;
    public DcMotor intakeWheels = null;
    public Servo servoClaw = null;
    public DcMotor elevator = null;
    public CRServo servoIntake = null;
    public CRServo servoLoop = null;
    private float armAngle = 0;
    public DcMotor arm = null;
    public DigitalChannel digIn0;
    public float elevator_zero_position=0;
    double vertical,horizontal,pivot = 0.0;

    StudShooter shooter = new StudShooter();

    public void init(HardwareMap hardwareMap) {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftRearDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator  = hardwareMap.get(DcMotor.class, "elevator");
        elevator.setDirection(DcMotor.Direction.FORWARD);


        digIn0 = hardwareMap.get(DigitalChannel.class, "switch0"); //false is not pressed
        digIn0.setMode(DigitalChannel.Mode.INPUT);

        //intakeBelt  = hardwareMap.get(DcMotor.class, "intake_belt");
        intakeWheels  = hardwareMap.get(DcMotor.class, "intake_wheels");
        arm  = hardwareMap.get(DcMotor.class, "arm");

        //intakeBelt.setDirection(DcMotor.Direction.FORWARD);
        intakeWheels.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);

        servoIntake  = hardwareMap.crservo.get("servo_intake");
        intakeBelt  = hardwareMap.crservo.get("intake_belt");
        servoLoop  = hardwareMap.crservo.get("servo_loop");



        servoClaw  = hardwareMap.servo.get("servo_claw");
        servoClaw.setPosition(0.3);
        shooter.init(hardwareMap);

    }

    public void my_sleep(float ms){
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void init_elevator(){
        //going down

        elevator.setPower(0.4);
        my_sleep(300);


        //going up
        elevator.setPower(-0.4);
        do{
            my_sleep(10);
        }while(!digIn0.getState());
        elevator.setPower(0);

        elevator_zero_position=elevator.getCurrentPosition();
    }
    public StudShooter getShooter() {
        return shooter;
    }

}
