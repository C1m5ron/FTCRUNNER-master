package org.firstinspires.ftc.teamcode.New_Stuff;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="OPEN HOUSE", group="TeleOp")
public class OH_TeleOp extends OpMode {
    private DcMotor motor;
    private DcMotor motor2;

    private DcMotor Left_Pullup;

    private DcMotor Right_Pullup;

    private Servo Drone_SERVO;

    private Servo Bucket_SERVO;

    private DcMotor backleftDrive = null;
    private DcMotor frontleftDrive = null;
    private DcMotor backrightDrive = null;
    private DcMotor frontrightDrive = null;

    private static final double COUNTS_PER_ROTATION = 751.8; //put the counts per rotation here. Can be found on servo city website

    double Power = .25;


    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "LiftMotor1");
        Left_Pullup = hardwareMap.get(DcMotor.class, "LPU");
        Right_Pullup = hardwareMap.get(DcMotor.class, "RPU");

        Bucket_SERVO = hardwareMap.get(Servo.class, "B_SERVO");
        Drone_SERVO = hardwareMap.get(Servo.class, "D_SERVO");


        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Pullup.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Left_Pullup.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Left_Pullup.setDirection(DcMotor.Direction.REVERSE);
        Left_Pullup.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Pullup.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right_Pullup.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right_Pullup.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        backleftDrive  = hardwareMap.get(DcMotor.class, "bld");
        frontleftDrive = hardwareMap.get(DcMotor.class, "fld");
        backrightDrive = hardwareMap.get(DcMotor.class, "brd");
        frontrightDrive = hardwareMap.get(DcMotor.class, "frd");

        frontrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        backleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Setup a variable for each drive wheel to save power level for telemetry
        double lfPower;
        double rfPower;
        double lbPower;
        double rbPower;


        double drive = gamepad1.left_stick_y;  // Negative to invert forward/backward
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        lfPower = Range.clip(drive + strafe + turn, -1.0, 1.0);
        rfPower = Range.clip(drive - strafe - turn, -1.0, 1.0);
        lbPower = Range.clip(drive - strafe + turn, -1.0, 1.0);
        rbPower = Range.clip(drive + strafe - turn, -1.0, 1.0);

        // Send calculated power to each wheel
        frontleftDrive.setPower(lfPower);
        frontrightDrive.setPower(rfPower);
        backleftDrive.setPower(lbPower);
        backrightDrive.setPower(rbPower);

        if (gamepad1.a) {
            double targetPosition1 = COUNTS_PER_ROTATION * 2.2;
            motor.setTargetPosition((int) targetPosition1);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(Power);
        } else if (gamepad1.x) {
            double targetPosition1 = COUNTS_PER_ROTATION * .1;
            motor.setTargetPosition((int) targetPosition1);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(Power);
        } else if (gamepad1.dpad_up) {
            double targetPosition2 = COUNTS_PER_ROTATION * 13.5;
            Left_Pullup.setTargetPosition((int) targetPosition2);
            Right_Pullup.setTargetPosition((int) targetPosition2);
            Left_Pullup.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Right_Pullup.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Left_Pullup.setPower(1);
            Right_Pullup.setPower(1);
        }
        else if (gamepad1.dpad_down){

            Left_Pullup.setTargetPosition(0);
            Right_Pullup.setTargetPosition(0);
            Left_Pullup.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Right_Pullup.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Left_Pullup.setPower(1);
            Right_Pullup.setPower(1);
        }
        else if (gamepad1.left_bumper){
            Bucket_SERVO.setPosition(.2);
        }
        else if (gamepad1.right_bumper){
            Bucket_SERVO.setPosition(0);
        }
        else if (gamepad1.dpad_left){
            Drone_SERVO.setPosition(1);
        }
        else {
            Drone_SERVO.setPosition(0);
        }


        while (motor.isBusy()) {
            telemetry.addData("Current Position Lift", motor.getCurrentPosition());
            telemetry.addData("Current Position Pullup", Left_Pullup.getCurrentPosition());
            telemetry.addData("Target Lift Position", motor.getTargetPosition());
            telemetry.addData("Target Pullup Position", Left_Pullup.getTargetPosition());
            telemetry.update();
        }
        // motor.setPower(0);
        //motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}


