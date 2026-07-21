package org.firstinspires.ftc.teamcode.Testing;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Testing Positions", group="Tele")
@Configurable
public class testingPositions extends OpMode {

    public DcMotorEx turretMotor;
    public DcMotorEx spinMotor;

    private DcMotorEx intakeMotor;
    private DcMotorEx popperMotor;

    private Servo transferServo;
    private Servo hoodServo;

    private Limelight3A limelight;

    public static double hoodTargetPos = 0;
    public static double turretPow = 0;
    public static double spinVelo = 0;

    public static double intakePow = 0;
    public static double popperPow = 0;
    public static double transferPos = 0;

    public static int turretPos = 0;

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    DigitalChannel intakeBeam;
    DigitalChannel outtakeBeam;

    Servo underglow;

    public void init() {
        //con 1
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        //exp 3
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        //con 3
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        //con 2
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);


        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        //exp 2
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setPower(0);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //con 0
        spinMotor = hardwareMap.get(DcMotorEx.class, "spinMotor");
        spinMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        spinMotor.setVelocity(0);

        //exp 1
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        intakeMotor.setPower(0);

        //exp 0
        popperMotor = hardwareMap.get(DcMotorEx.class, "popperMotor");
        popperMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        popperMotor.setPower(0);

        //exp servo 0
        transferServo = hardwareMap.get(Servo.class, "transferServo");
        transferServo.setPosition(transferServo.getPosition());

        //con servo 0
        //Top: 0
        //Bottom: 0.71
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        hoodServo.setPosition(hoodServo.getPosition());
        hoodServo.setDirection(Servo.Direction.REVERSE);

        pidfStart = spinMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

//        intakeBeam = hardwareMap.get(DigitalChannel.class, "intakeBeam");
//        intakeBeam.setMode(DigitalChannel.Mode.INPUT);

        outtakeBeam = hardwareMap.get(DigitalChannel.class, "outtakeBeam");
        outtakeBeam.setMode(DigitalChannel.Mode.INPUT);

        underglow = hardwareMap.get(Servo.class, "underglow");
        underglow.setPosition(underglow.getPosition());
    }

    PIDFCoefficients pidfStart;

    public static double p = 0;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0;

    public static double targetPow = 0;

    public static boolean velocity = true;


    public double hoodPos = 0;

    LLResult results;
    public void loop(){
        underglow.setPosition(0.708);

        hoodControl();

        results = limelight.getLatestResult();

        drivetrain();

        if(velocity) {
            spinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            PIDFCoefficients pidf = new PIDFCoefficients(p, i, d, f, MotorControlAlgorithm.PIDF);

//        spinMotor.setVelocityPIDFCoefficients(p, i, d, f);
            spinMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

            spinMotor.setVelocity(spinVelo);
        }else{
            spinMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            spinMotor.setPower(targetPow);
        }

        hoodServo.setPosition(hoodPos);

        if(gamepad1.leftBumperWasPressed()){
            popperPow = 0.6;
            intakePow = 1;
            transferPos = 0.72;
        }
        if(gamepad1.rightBumperWasPressed()){
            popperPow = 1;
            intakePow = 1;
            transferPos = 0.42;
        }

        //off: 0.72
        //on: 0.42
        transferServo.setPosition(transferPos);

        popperMotor.setPower(popperPow);
        intakeMotor.setPower(intakePow);
        turretMotor.setPower(turretPow);
        turretMotor.setTargetPosition(turretPos);


        telemetry.addData("distance", limelight.getLatestResult().getBotposeAvgDist());
        telemetry.addData("something there", outtakeBeam.getState());
        telemetry.addLine();
        telemetry.addData("turretPosition", turretMotor.getCurrentPosition());
        telemetry.addData("tx", results.getTx());
        telemetry.addLine();
        telemetry.addData("p: ", pidfStart.p);
        telemetry.addData("f: ", pidfStart.f);
        telemetry.addData("targetVelo", spinVelo);
        telemetry.addData("currentVelo", spinMotor.getVelocity());
        telemetry.addLine();
        telemetry.addData("Hood Position - ", hoodServo.getPosition());
        telemetry.addData("Transfer Position - ", transferServo.getPosition());
        telemetry.addLine();
        telemetry.addData("Popper Power - ", popperMotor.getPower());
        telemetry.addData("Intake Power - ", intakeMotor.getPower());
        telemetry.addData("Turret Power - ", turretMotor.getPower());
        telemetry.addData("Spin Velocity - ", spinMotor.getVelocity());
        telemetry.update();
    }
    private void drivetrain(){
        //Drivetrain
        double moveX = gamepad1.left_stick_x;
        double moveY = -gamepad1.left_stick_y;
        double turnX = gamepad1.right_stick_x;

        double frontLeftPower = (moveY + moveX + turnX);
        double frontRightPower = (moveY - moveX - turnX);
        double backLeftPower = (moveY - moveX + turnX);
        double backRightPower = (moveY + moveX - turnX);

        //Drivetrain Driver Controls
        if(Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.right_stick_x) > 0.1){

            if(gamepad1.right_bumper){
                leftFront.setPower(frontLeftPower * 0.9);
                rightFront.setPower(frontRightPower * 0.9 );
                leftBack.setPower(backLeftPower * 0.9);
                rightBack.setPower(backRightPower * 0.9);
            }else if(gamepad1.left_bumper){
                leftFront.setPower(frontLeftPower * 0.35);
                rightFront.setPower(frontRightPower * 0.35);
                leftBack.setPower(backLeftPower * 0.35);
                rightBack.setPower(backRightPower * 0.35);
            }else{
                leftFront.setPower(frontLeftPower * 0.75);
                rightFront.setPower(frontRightPower * 0.75);
                leftBack.setPower(backLeftPower * 0.75);
                rightBack.setPower(backRightPower * 0.75);
            }
        }else{
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
        }
    }

    public static double a = 1;
    public static double b = 0;

    private void hoodControl(){
        //find regression for hoodTarget based on velocity
//            hoodTargetPos = (1 * spinVelo);

        int veloError = (int)(spinMotor.getVelocity() - spinVelo);

        //find regression between the error in velocity and relating hood pos required
        hoodPos = hoodTargetPos + ((veloError * a) + b);
        if(hoodPos > 0.58)
            hoodPos = 0.58;
        if(hoodPos < 0.28)
            hoodPos = 0.28;
    }


}
