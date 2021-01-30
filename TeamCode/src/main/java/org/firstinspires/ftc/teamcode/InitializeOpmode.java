package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Sarthak on 10/4/2019.
 */
@TeleOp(name = "Initialize OpMode")
public class InitializeOpmode extends LinearOpMode {

    StudBot studbot = new StudBot();

    @Override
    public void runOpMode() throws InterruptedException {

        studbot.init(hardwareMap);

        waitForStart();
        boolean abc = false;
        //shooterUpdate.maintainSpeedModeStart(0.9);
        while (opModeIsActive()) {
            if (!abc) {
                studbot.getElevator().findZeroPoint();
                abc = true;
            }

            telemetry.addData("switch0", studbot.getElevator().digIn0.getState());
            telemetry.addData("switch1=", studbot.getArm().digIn1.getState());
            telemetry.addData("elevator=", studbot.getElevator().getElevatorPosition() - studbot.getElevator().elevator_zero_position);
            telemetry.update();

        }

        studbot.getIntake().stopAll();

    }


}
