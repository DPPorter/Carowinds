package org.firstinspires.ftc.teamcode.Testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;
@Configurable
@TeleOp
public class regressionBuilder extends OpMode {
    public DcMotorEx turretMotor;
    public DcMotorEx flywheelMotor;
    public DcMotorEx intakeMotor;
    public DcMotorEx popperMotor;

    public Servo popServo;
    public Servo hoodServo;
//    public Servo topServo;

    public Limelight3A limelight;
    public Servo underglow;
    public DigitalChannel intakeBeam;
    public DigitalChannel outtakeBeam;


    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;


    PIDFCoefficients pidVariables = new PIDFCoefficients(250, 0, 0, 17.7, MotorControlAlgorithm.PIDF);

    public enum RobotStates {INTAKE, REST, SHOOT, STOP};
    RobotStates state = RobotStates.STOP;

    private enum ShootStates {SET, FIRE}
    ShootStates shootState = ShootStates.SET;

    ElapsedTime intakeTimer = new ElapsedTime();
    ElapsedTime popTimer = new ElapsedTime();

    boolean farLock = false;
    boolean red = true;

    @Override
    public void init() {

        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turretMotor.setTargetPosition(turretMotor.getCurrentPosition());
        turretMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0.4);

        flywheelMotor = hardwareMap.get(DcMotorEx.class, "spinMotor");
        flywheelMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidVariables);
        flywheelMotor.setVelocity(0);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(0);

        popperMotor = hardwareMap.get(DcMotorEx.class, "popperMotor");
        popperMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        popperMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        popperMotor.setPower(0);


        popServo = hardwareMap.get(Servo.class, "transferServo");
        popServo.setPosition(popServo.getPosition());

        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        hoodServo.setDirection(Servo.Direction.REVERSE);

//        topServo = hardwareMap.get(Servo.class, "topServo");
//        topServo.setPosition(topServo.getPosition());


        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(0);

        underglow = hardwareMap.get(Servo.class, "underglow");
        underglow.setPosition(underglow.getPosition());

        intakeBeam = hardwareMap.get(DigitalChannel.class, "intakeBeam");
        intakeBeam.setMode(DigitalChannel.Mode.INPUT);

        outtakeBeam = hardwareMap.get(DigitalChannel.class, "outtakeBeam");
        outtakeBeam.setMode(DigitalChannel.Mode.INPUT);



        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(54, 94))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(135), 0.8))
                .build();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();
        llReset();
        velocityControl();
        turretControl();
        drivetrain();

        if(!gamepad1.right_bumper){
            intakeMotor.setPower(1);
            popperMotor.setPower(1);

            popServo.setPosition(0.4);
//            topServo.setPosition(0.45);
        }else{
            intakeMotor.setPower(1);
            popperMotor.setPower(1);

//            topServo.setPosition(0.6);

            if(gamepad1.leftBumperWasPressed()) popTimer.reset();

            if(popTimer.milliseconds() < 150) popServo.setPosition(0.4);
            else popServo.setPosition(0.21);
        }


        if(gamepad2.b){
            limelight.pipelineSwitch(0);
            red = true;
        }else if(gamepad2.x){
            limelight.pipelineSwitch(2);
            red = false;
        }

        if(red) telemetry.addLine("RED");
        else telemetry.addLine("BLUE");

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addLine();
        telemetry.addData("distance", dist);
        telemetry.addData("velocity", flywheelMotor.getVelocity());
        telemetry.addData("veloError", veloError);
        telemetry.addLine();
        telemetry.addData("turretError", turretError);
    }

    public static double goalX = 0;
    public static double goalY = 142;

    int turretError = 0;
    private void turretControl(){
        double turretToCenter = 3.325;

        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();

        double robotRot = follower.getPose().getHeading();

        double turretX = robotX - (turretToCenter * Math.cos(robotRot));
        double turretY = robotY - (turretToCenter * Math.sin(robotRot));

        double xDiff = goalX - turretX;
        double yDiff = goalY - turretY;

        double targetAngle = Math.toDegrees(Math.atan2(yDiff, xDiff));

        robotRot = Math.toDegrees(robotRot);
        if (robotRot < 0)
            robotRot += 360;
        if (robotRot >= 360)
            robotRot -= 360;

        targetAngle = (robotRot - targetAngle);

        if (targetAngle < -180)
            targetAngle += 360;
        if (targetAngle > 180)
            targetAngle -= 360;

        double a = -2.99525;
        double b = 0.353539;
        int turretTarget = (int)((targetAngle * a) + b);

        int trueTarget = turretTarget;

        if (turretTarget < -200)
            turretTarget = -200;
        if (turretTarget > 200)
            turretTarget = 200;


        turretMotor.setTargetPosition(0);

        turretError = turretMotor.getCurrentPosition() - trueTarget;

        int turretErrorAbs = Math.abs(turretError);
        if(turretErrorAbs > 150) turretMotor.setPower(0.25);
        else if(turretErrorAbs > 50) turretMotor.setPower(0.4);
        else turretMotor.setPower(0.55);
    }

    int veloError = 0;

    public static int target = 0;

    public double dist;

    private void velocityControl(){

        double distance = Math.sqrt((Math.pow((goalX - follower.getPose().getX()), 2) + Math.pow((goalY - follower.getPose().getY()), 2)));

        dist = distance;

        flywheelMotor.setVelocity(target);
        veloError = Math.abs((int)(flywheelMotor.getVelocity() - target));

        hoodControl();
    }

    public static double a = 0.52;
    public static double b = 1.0005;

    public static double highHood = 0.58;
    public static double lowHood = 0.52;
    private void hoodControl(){

            double hoodTarget = a * Math.pow(b, veloError);

            if (hoodTarget > highHood)
                hoodTarget = highHood;
            if (hoodTarget < lowHood)
                hoodTarget = lowHood;


        hoodServo.setPosition(hoodTarget);
    }

    private void llReset(){
        LLResult results = limelight.getLatestResult();

        Pose2D ftcPose2d = new Pose2D(DistanceUnit.INCH, (results.getBotpose().getPosition().x * 39.3701), (results.getBotpose().getPosition().y * 39.3701), AngleUnit.RADIANS, AngleUnit.normalizeRadians(results.getBotpose().getOrientation().getYaw(AngleUnit.RADIANS)));

        Pose ftcStandard = PoseConverter.pose2DToPose(ftcPose2d, FTCCoordinates.INSTANCE);
        Pose current = ftcStandard.getAsCoordinateSystem(PedroCoordinates.INSTANCE);


        if(results.getBotposeAvgDist() != 0) follower.setPose(current);;
    }

    private void drivetrain(){
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                (-0.00527008 * turretError),
                false // field Centric
        );

    }
}

