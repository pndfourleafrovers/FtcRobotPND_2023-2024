package org.firstinspires.ftc.teamcode.Objects;
import static org.firstinspires.ftc.teamcode.Objects.Odometry.DRIVE_SPEED;
import static org.firstinspires.ftc.teamcode.Objects.Odometry.TURN_SPEED;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Disabled
@Autonomous
public class RedProp {
    Odometry OdoMethods = new Odometry();
    int Position;
    private ColorSensor leftColor, rightColor;

    public void ambient() {
        int leftAmbient = leftColor.alpha();
        int rightAmbient = rightColor.alpha();

        if (leftAmbient > 75) { // Adjust the value based on your observations and testing
            Position = 1;
        } else if (rightAmbient > 120) { // Adjust the value based on your observations and testing
            Position = 2;
        } else if (leftAmbient < 75 && rightAmbient < 120) {
            Position = 3;
        }
    }

    /*
    public void propBack (){
//Make holdTime a variable
        switch (Position) {
            case 1:
                OdoMethods.driveStraight(DRIVE_SPEED, -20, 0.0);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, 30);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, 30, 0.25);

                OdoMethods.driveStraight(DRIVE_SPEED, 10, 30);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, 30);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, 0, 0.25);
                //Grab code
                OdoMethods.driveStraight(DRIVE_SPEED, -10, 30);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, 0);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, 0, 0.25);

                OdoMethods.driveStraight(DRIVE_SPEED, 40, 0);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, 0);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, 0, 0.25);
                break;
            case 2:
                OdoMethods.driveStraight(DRIVE_SPEED, -20, 0.0);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, -30);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, -30, 0.25);

                OdoMethods.driveStraight(DRIVE_SPEED, 10, -30);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, -30);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, -30, 0.25);
                // Grab code
                OdoMethods.driveStraight(DRIVE_SPEED, -10, -30);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, 0);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, 0, 0.25);

                OdoMethods.driveStraight(DRIVE_SPEED, 40, 0);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, 0);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, 0, 0.25);
                break;
            case 3:
                OdoMethods.driveStraight(DRIVE_SPEED, -20, 0.0);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, 10);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, 10, 0.25);

                OdoMethods.driveStraight(DRIVE_SPEED, 30, 10);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, 10);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, 10, 0.25);
                //Grab code
                OdoMethods.driveStraight(DRIVE_SPEED, -30, 30);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, 90);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, 90, 0.25);

                OdoMethods.driveStraight(DRIVE_SPEED, 25, 90);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, 0);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, 0, 0.25);

                OdoMethods.driveStraight(DRIVE_SPEED, 40, 0);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, -90);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, -90, 0.25);

                OdoMethods.driveStraight(DRIVE_SPEED, 25, -90);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, -90);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, -90, 0.25);
                break;
                //By the end of each sequence, each movement is in same proximate position somewhere above the middle spike.
        }
        }
     */
    public void propUpFront() {
//Make holdTime a variable
        switch (Position) {
            case 1:
                OdoMethods.driveStraight(DRIVE_SPEED, -20, 0.0);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, 30);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, 30, 0.25);

                OdoMethods.driveStraight(DRIVE_SPEED, 10, 30);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, 30);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, 0, 0.25);
                //Grab code
                OdoMethods.driveStraight(DRIVE_SPEED, -10, 30);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, -90);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, -90, 0.25);

                OdoMethods.driveStraight(DRIVE_SPEED, 20, -90);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, -90);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, -90, 0.25);
                break;
            case 2:
                OdoMethods.driveStraight(DRIVE_SPEED, -20, 0.0);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, -30);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, -30, 0.25);

                OdoMethods.driveStraight(DRIVE_SPEED, 10, -30);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, -30);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, -30, 0.25);
                // Grab code
                OdoMethods.driveStraight(DRIVE_SPEED, -10, -30);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, 0);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, 0, 0.25);

                OdoMethods.driveStraight(DRIVE_SPEED, 30, 0);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, -90);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, -90, 0.25);

                OdoMethods.driveStraight(DRIVE_SPEED, 20, -90);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, -180);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, -180, 0.25);

                OdoMethods.driveStraight(DRIVE_SPEED, 30, -180);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, -90);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, -90, 0.25);
                break;
            case 3:
                OdoMethods.driveStraight(DRIVE_SPEED, -20, 0.0);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, 10);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, 10, 0.25);

                OdoMethods.driveStraight(DRIVE_SPEED, 30, 10);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, 10);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, 10, 0.25);
                //Grab code
                OdoMethods.driveStraight(DRIVE_SPEED, -30, 30);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, -90);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, -90, 0.25);

                OdoMethods.driveStraight(DRIVE_SPEED, 20, -90);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, -90);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, -90, 0.25);
                break;
        }
    }

    public void propDownFront() {
//Make holdTime a variable
        switch (Position) {
            case 1:
                OdoMethods.driveStraight(DRIVE_SPEED, -20, 0.0);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, 30);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, 30, 0.25);

                OdoMethods.driveStraight(DRIVE_SPEED, 10, 30);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, 30);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, 0, 0.25);
                //Grab code
                OdoMethods.driveStraight(DRIVE_SPEED, -10, 30);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, -90);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, -90, 0.25);

                OdoMethods.driveStraight(DRIVE_SPEED, 20, -90);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, -90);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, -90, 0.25);
                break;
            case 2:
                OdoMethods.driveStraight(DRIVE_SPEED, -20, 0.0);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, -30);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, -30, 0.25);

                OdoMethods.driveStraight(DRIVE_SPEED, 10, -30);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, -30);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, -30, 0.25);
                // Grab code
                OdoMethods.driveStraight(DRIVE_SPEED, -10, -30);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, -180);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, -180, 0.25);

                OdoMethods.driveStraight(DRIVE_SPEED, 30, -180);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, -90);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, -90, 0.25);

                OdoMethods.driveStraight(DRIVE_SPEED, 20, -90);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, 0);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, 0, 0.25);

                OdoMethods.driveStraight(DRIVE_SPEED, 30, 0);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, -90);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, -90, 0.25);
                break;
            case 3:
                OdoMethods.driveStraight(DRIVE_SPEED, -20, 0.0);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, 10);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, 10, 0.25);

                OdoMethods.driveStraight(DRIVE_SPEED, 30, 10);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, 10);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, 10, 0.25);
                //Grab code
                OdoMethods.driveStraight(DRIVE_SPEED, -30, 30);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, -90);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, -90, 0.25);

                OdoMethods.driveStraight(DRIVE_SPEED, 20, -90);    // In inches
                OdoMethods.turnToHeading(TURN_SPEED, -90);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, -90, 0.25);
                break;
        }
    }

    public void propSide() {
        OdoMethods.driveStraight(DRIVE_SPEED, -20, 0.0);    // In inches
        OdoMethods.turnToHeading(TURN_SPEED, 30);               // In Degrees
        OdoMethods.holdHeading(TURN_SPEED, 30, 0.25);

        OdoMethods.driveStraight(DRIVE_SPEED, 10, 30);    // In inches
        OdoMethods.turnToHeading(TURN_SPEED, 30);               // In Degrees
        OdoMethods.holdHeading(TURN_SPEED, 0, 0.25);
        //Grab code
        OdoMethods.driveStraight(DRIVE_SPEED, -10, 30);    // In inches
        OdoMethods.turnToHeading(TURN_SPEED, -180);               // In Degrees
        OdoMethods.holdHeading(TURN_SPEED, -180, 0.25);

        OdoMethods.driveStraight(DRIVE_SPEED, 40, -190);    // In inches
        OdoMethods.turnToHeading(TURN_SPEED, 0);               // In Degrees
        OdoMethods.holdHeading(TURN_SPEED, 0, 0.25);
    }
}
