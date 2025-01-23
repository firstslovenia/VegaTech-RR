package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "TankVegaTech Stuttgart")
public class TankVegaTech extends LinearOpMode {
    private DcMotor elbow;
    private AnalogInput elbow_angle;
    private CRServo dlan;
    private Servo zapestje;

    double elbowPotenciometer;
    int prsti;
    double elbowPower;
    double elbowTargetPos;
    int maxElbowPosition;
    int minElbowPosition;
    int elbowMin;
    int elbowMax;
    double zapestjeMin;
    double zeljenaPozicija;
    double handMin;
    double handMax;
    double handWantedPosition;

    /**
     * This OpMode offers Tank Drive style TeleOp control for a direct drive robot.
     * <p>
     * In this Tank Drive mode, the left and right joysticks (up
     * and down) drive the left and right motors, respectively.
     */
    @Override
    public void runOpMode() {
        int zapestjeTurnNeki;

        DcMotor right_drive = hardwareMap.get(DcMotor.class, "right_drive");
        elbow = hardwareMap.get(DcMotor.class, "elbow");
        DcMotor left_drive = hardwareMap.get(DcMotor.class, "left_drive");
        DcMotor strafe_motor = hardwareMap.get(DcMotor.class, "strafe_motor");
        elbow_angle = hardwareMap.get(AnalogInput.class, "elbow_angle");
        dlan = hardwareMap.get(CRServo.class, "dlan");
        zapestje = hardwareMap.get(Servo.class, "zapestje");

        // Reverse one of the drive motors.
        // You will have to determine which motor to reverse for your robot.
        // In this example, the right motor was reversed so that positive
        // applied power makes it move the robot in the forward direction.
        right_drive.setDirection(DcMotor.Direction.REVERSE);
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        // ZapestjeMax se ni testirano.
        elbowTargetPos = 100;
        elbowPotenciometer = 0;
        zapestjeTurnNeki = 0;
        elbowMin = 12;
        elbowMax = 270;
        // ZapestjeMin se ni testirano.
        zapestjeMin = -0.1;
        // ZapestjeMax se ni testirano.
        prsti = 0;
        elbowPower = 0;
        maxElbowPosition = 40;
        minElbowPosition = 260;
        zeljenaPozicija = maxElbowPosition;
        handMin = 0;
        handMax = 1;
        handWantedPosition = 1;


        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                // The Y axis of a joystick ranges from -1 in its topmost position to +1 in its bottommost position.
                // We negate this value so that the topmost position corresponds to maximum forward power.
                left_drive.setPower(-gamepad1.left_stick_y);
                right_drive.setPower(-gamepad1.right_stick_y);
                strafe_motor.setPower(-gamepad1.left_stick_x);

                elbowMovement2();
                hand2();
                prstki();
                rokaPosSet();

                telemetry.addData("Left Pow", left_drive.getPower());
                telemetry.addData("Right Pow", right_drive.getPower());
                telemetry.addData("Potenciometer angle", elbowPotenciometer);
                telemetry.addData("Zapestje angle", zapestjeTurnNeki);
                // Mozno je da bo stevilka za zapestjeTurnNeki drugacna ce bo odloceno za uporabo druge vrste servo motorja.
                telemetry.addData("elbow angle", elbow_angle.getVoltage() * 81.8);

                telemetry.update();
            }
        }
    }

    /**
     * Describe this function...
     */
    private void prstki() {
        telemetry.addData("test", "test");
        if (gamepad2.left_trigger > 0.7) {
            prsti = 1;
            telemetry.addData("prsti", prsti);
        } else if (gamepad2.right_trigger > 0.7) {
            prsti = -1;
            telemetry.addData("prsti", prsti);
        } else {
            telemetry.addData("prsti", prsti);
        }
        dlan.setPower(prsti);
    }

    /**
     * right -> up
     * left -> down
     */
    private void elbowMovement2() {
        double elbowPower;
        double elbowCurrentAngle2;
        double elbowP = 0.03;

        elbowCurrentAngle2 = elbow_angle.getVoltage() * 81.8;
        telemetry.addData("elbow current angle", elbowCurrentAngle2);


        telemetry.addData("left stick y", -gamepad2.left_stick_y);
        zeljenaPozicija += gamepad2.left_stick_y * 0.2;
        if (zeljenaPozicija < maxElbowPosition) {
            telemetry.addData("ces", "dol");
            zeljenaPozicija = maxElbowPosition;
        } else if (zeljenaPozicija > minElbowPosition) {
            telemetry.addData("ces", "gor");
            zeljenaPozicija = minElbowPosition;
        }

        elbowPower = -((zeljenaPozicija - elbowCurrentAngle2) * elbowP);

        telemetry.addData("elbow power", elbowPower);
        telemetry.addData("zeljena pozicija", zeljenaPozicija);

        elbow.setPower(elbowPower);
        telemetry.addData("elbow power", elbowPower);
        telemetry.addData("calculation", (elbowCurrentAngle2 - maxElbowPosition) / 40);
    }

    private void hand2() {
        handWantedPosition += gamepad2.right_stick_y * 0.003;
        if (handWantedPosition > handMax) {
            handWantedPosition = handMax;
        } else if (handWantedPosition < handMin) {
            handWantedPosition = handMin;
        }

        zapestje.setPosition(handWantedPosition);

        telemetry.addData("zelena pozicija zapestja", handWantedPosition);

    }

    private void rokaPosSet() {
        if (gamepad2.circle) {
            zeljenaPozicija = maxElbowPosition;
            handWantedPosition = handMax;
        } else if (gamepad2.square) {
            zeljenaPozicija = 205;
            handWantedPosition = handMin;
        } else if (gamepad2.triangle) {
            zeljenaPozicija = 200;
            handWantedPosition = 0.55;
        } else if (gamepad2.cross) {
            zeljenaPozicija = 110;
            handWantedPosition = 0.2;
        }
    }
}
