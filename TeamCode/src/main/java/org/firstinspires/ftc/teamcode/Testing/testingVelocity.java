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
public class testingVelocity extends OpMode {
    public DcMotorEx turretMotor;
    public DcMotorEx flywheelMotor;
    public DcMotorEx intakeMotor;
    public DcMotorEx popperMotor;

    public Servo popServo;
    public Servo hoodServo;
    public Servo topServo;

    public Limelight3A limelight;
    public Servo underglow;
    public DigitalChannel intakeBeam;
    public DigitalChannel outtakeBeam;


    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;

    ElapsedTime popTimer = new ElapsedTime();



    PIDFCoefficients pidVariables = new PIDFCoefficients(250, 0, 0, 17.7, MotorControlAlgorithm.PIDF);

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

        topServo = hardwareMap.get(Servo.class, "topServo");
        topServo.setPosition(topServo.getPosition());


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

        if(gamepad2.b){
            goalX = 142;
            limelight.pipelineSwitch(0);
            red = true;
        }else if(gamepad2.x){
            goalX = 0;
            limelight.pipelineSwitch(2);
            red = false;
        }

        if(gamepad1.right_bumper){
            intakeMotor.setPower(1);
            popperMotor.setPower(1);

            topServo.setPosition(0.6);

            if(gamepad1.leftBumperWasPressed()) popTimer.reset();

            if(popTimer.milliseconds() < 100) popServo.setPosition(0.4);
            else popServo.setPosition(0.21);

            underglow.setPosition(0.444);
        }else{
            intakeMotor.setPower(1);
            popperMotor.setPower(0.8);

            topServo.setPosition(0.45);

            if(gamepad1.leftBumperWasPressed()) popTimer.reset();

            if(popTimer.milliseconds() < 100) popServo.setPosition(0.4);
            else popServo.setPosition(0.21);

            underglow.setPosition(0.444);
        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);

        if(red) telemetryM.addLine("RED");
        else telemetryM.addLine("BLUE");
    }

    double goalX = 0;
    double goalY = 142;

    int veloError = 0;
    private void velocityControl(){

        double distance = Math.sqrt((Math.pow((goalX - follower.getPose().getX()), 2) + Math.pow((goalY - follower.getPose().getY()), 2)));

        boolean close = distance <= 70;

        int target;
        if(close) target = (int)((1.97937 * distance) + 1156.16893);
        else target = (int)((-0.117108 * Math.pow(distance, 2)) + (17.9503 * distance) + 406.97247);

        if(farLock && target < 1300) target = 1300;

        flywheelMotor.setVelocity(target);
        veloError = Math.abs((int)(flywheelMotor.getVelocity() - target));

        hoodControl(close);
    }

    private void hoodControl(boolean close){

        double hoodTarget;

        if(!close){
            hoodTarget = 0.52 * Math.pow(1.0005, veloError);

            if (hoodTarget > 0.58)
                hoodTarget = 0.58;
            if (hoodTarget < 0.52)
                hoodTarget = 0.52;
        }else{
            hoodTarget = 0.8 * Math.pow(0.996, veloError);

            if (hoodTarget > 0.8)
                hoodTarget = 0.8;
            if (hoodTarget < 0.58)
                hoodTarget = 0.58;
        }

        hoodServo.setPosition(hoodTarget);
    }

    private void llReset(){
        LLResult results = limelight.getLatestResult();

        Pose2D ftcPose2d = new Pose2D(DistanceUnit.INCH, (results.getBotpose().getPosition().x * 39.3701), (results.getBotpose().getPosition().y * 39.3701), AngleUnit.RADIANS, AngleUnit.normalizeRadians(results.getBotpose().getOrientation().getYaw(AngleUnit.RADIANS)));

        Pose ftcStandard = PoseConverter.pose2DToPose(ftcPose2d, InvertedFTCCoordinates.INSTANCE);
        Pose current = ftcStandard.getAsCoordinateSystem(PedroCoordinates.INSTANCE);


        if(results.getBotposeAvgDist() != 0 && Math.abs(results.getTx()) < 10) follower.setPose(current);;
    }
}

