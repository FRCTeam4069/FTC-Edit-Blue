package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
@Disabled
public class Edit_Blue extends LinearOpMode {

    // controls
    double speed_multiplier = 1;

    double backward = gamepad1.left_trigger;
    double forward = gamepad1.right_trigger;
    double turn = gamepad1.right_stick_x;
    double ElevatorPower = booleanAsDouble(gamepad1.dpad_up, gamepad1.dpad_down, 0.7, 0.7, 0);


    // Names of Motors
    String leftmotor = "leftmotor";
    DcMotorSimple.Direction leftmotorDirection = DcMotorSimple.Direction.REVERSE;

    String rightmotor = "rightmotor";
    DcMotorSimple.Direction rightmotorDirection = DcMotorSimple.Direction.FORWARD;

    String elevatorR = "elevatorRight";
    DcMotorSimple.Direction elevatorRDirection = DcMotorSimple.Direction.FORWARD;

    String elevatorL = "elevatorLeft";
    DcMotorSimple.Direction elevatorLDirection = DcMotorSimple.Direction.FORWARD;

    String coneGrabberThingLeft = "coneGrabberThingLeft";
    DcMotorSimple.Direction coneGrabberThingLeftDirection = DcMotorSimple.Direction.FORWARD;

    String coneGrabberThingRight = "coneGrabberThingRight";
    DcMotorSimple.Direction coneGrabberThingRightDirection = DcMotorSimple.Direction.FORWARD;


    // Names of Servos
    String CRServoRight = "rightWheel";
    CRServo.Direction CRServoRightDirection = CRServo.Direction.FORWARD;
    String CRServoLeft = "leftWheel";
    CRServo.Direction CRServoLefttDirection = CRServo.Direction.FORWARD;

    String ElevatorbaseLeft = "leftBase";
    String ElevatorbaseRight = "rightBase";
    String interestingDesignedBlueConeGrabber = "elevatorServo";

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor elevatorLeft = null;
    private DcMotor elevatorRight = null;
    private DcMotor backMechcanismRight = null;
    private DcMotor backMechcanismLeft = null;

    private CRServo BackIntakeRight = null;
    private CRServo BackIntakeLeft = null;
    private Servo EServoLeft = null;
    private Servo EServoRight = null;
    private Servo EServoMid = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
                    // -- Motors -- \\
        leftDrive  = hardwareMap.get(DcMotor.class, leftmotor);
        leftDrive.setDirection(leftmotorDirection);
        rightDrive = hardwareMap.get(DcMotor.class, rightmotor);
        rightDrive.setDirection(rightmotorDirection);
        elevatorLeft = hardwareMap.get(DcMotor.class, elevatorL);
        elevatorLeft.setDirection(elevatorLDirection);
        elevatorRight = hardwareMap.get(DcMotor.class, elevatorR);
        elevatorRight.setDirection(elevatorRDirection);
        backMechcanismRight = hardwareMap.get(DcMotor.class, coneGrabberThingRight);
        backMechcanismRight.setDirection(coneGrabberThingRightDirection);
        backMechcanismLeft = hardwareMap.get(DcMotor.class, coneGrabberThingLeft);
        backMechcanismLeft.setDirection(coneGrabberThingLeftDirection);
                    // -- Servos -- \\
        BackIntakeLeft = hardwareMap.get(CRServo.class, CRServoLeft);
        BackIntakeLeft.setDirection(CRServo.Direction.FORWARD);
        BackIntakeRight = hardwareMap.get(CRServo.class, CRServoRight);
        BackIntakeRight.setDirection(CRServo.Direction.FORWARD);
        EServoLeft = hardwareMap.get(Servo.class, ElevatorbaseLeft);
        EServoRight = hardwareMap.get(Servo.class, ElevatorbaseRight);
        EServoMid = hardwareMap.get(Servo.class, interestingDesignedBlueConeGrabber);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower =  Math.abs(speed_multiplier) * (forward - backward + turn);
            double rightPower = Math.abs(speed_multiplier) * (forward - backward - turn);
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            elevatorRight.setPower(ElevatorPower);
            elevatorLeft.setPower(ElevatorPower);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }

    /**
     * @param A button input that outputs max
     * @param B button input that outputs min
     * @apiNote if both A and B are true or false base will be returned
     * */
    public double booleanAsDouble(boolean A, boolean B, double max, double min, double base){
        if(A && B) return base;
        else if(A) return max;
        else if (B) return min;
        else return base;
    }
}
