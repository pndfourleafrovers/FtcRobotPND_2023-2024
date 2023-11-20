package org.firstinspires.ftc.teamcode.Archive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="MoveArm", group="Autonomous")
@Disabled
public class MoveArm extends LinearOpMode {

    DcMotor arm;

    enum State {
        MOVING,
        WAITING,
        READY_TO_MOVE
    }

    State state = State.READY_TO_MOVE;
    ElapsedTime timer = new ElapsedTime();
    static final int TICKS_PER_MOTOR_REV = 1425;
    final double TICKS_PER_DEGREE = TICKS_PER_MOTOR_REV / 120;
    final double WAIT_TIME = 1.0;       // Time to wait in seconds
    int moveArmToPosition = 0;          // Position to move the arm to in degrees

    @Override
    public void runOpMode() {
        arm = hardwareMap.get(DcMotor.class, "arm");

        waitForStart();

        while (opModeIsActive()) {
            moveArmToPosition = 90;  // Move arm to 45 degrees as an example
            controlArm(moveArmToPosition);


            // Add other robot controls here, they will continue to run while the arm is moving/waiting
        }
    }

    private void controlArm(int desiredPosition) {
        switch (state) {
            case READY_TO_MOVE:
                // Ready to move to a new position
                int targetPosition = (int) (TICKS_PER_DEGREE * desiredPosition);
                arm.setTargetPosition(targetPosition);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.5);
                state = State.MOVING;
                break;

            case MOVING:
                // Wait for the arm to reach the target position
                if (!arm.isBusy()) {
                    // Arm has reached the target, start the timer and switch to waiting state
                    timer.reset();
                    state = State.WAITING;
                }
                break;

            case WAITING:
                // Wait for the specified time
                if (timer.seconds() > WAIT_TIME) {
                    // Time is up, ready to move to a new position
                    state = State.READY_TO_MOVE;
                }
                break;
        }
    }
}
