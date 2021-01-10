package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class StudBot {

    private DcMotor leftFrontDrive = null;
    private DcMotor leftRearDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightRearDrive = null;

    private DcMotor intakeBelt = null;
    private DcMotor intakeWheels = null;

    private Servo servoIntake = null;
    private Servo servoLoop = null;
    private float armAngle = 0;
    private DcMotor arm = null;
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


        //intakeBelt  = hardwareMap.get(DcMotor.class, "intake_belt");
        intakeWheels  = hardwareMap.get(DcMotor.class, "intake_wheels");
        arm  = hardwareMap.get(DcMotor.class, "arm");

        //intakeBelt.setDirection(DcMotor.Direction.FORWARD);
        intakeWheels.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);

        servoIntake  = hardwareMap.servo.get("servo_intake");
        servoLoop  = hardwareMap.servo.get("servo_loop");

        shooter.init(hardwareMap);
    }


    public StudShooter getShooter() {
        return shooter;
    }

}
