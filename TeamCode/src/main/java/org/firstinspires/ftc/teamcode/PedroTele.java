package org.firstinspires.ftc.teamcode;
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
public class PedroTele extends OpMode {
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
        turretMotor.setPower(0.7);
        turretMotor.setPositionPIDFCoefficients(20);
        turretMotor.setVelocityPIDFCoefficients(25, 3, 0, 10);
        turretMotor.setTargetPositionTolerance(10);

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
        turretControl();
        drivetrain();

        switch(state){
            case INTAKE:
                intakeMotor.setPower(1);
                popperMotor.setPower(1);

                popServo.setPosition(0.4);
                topServo.setPosition(0.45);

                underglow.setPosition(0.694);

                int there = 0;
                for(int i = 0; i < 10; i ++) if(intakeBeam.getState()) there ++;
                if(there <= 1) intakeTimer.reset();

                if(intakeTimer.milliseconds() > 200){
                    state = RobotStates.REST;
                }

                break;
            case REST:
                intakeMotor.setPower(0.7);
                popperMotor.setPower(0.9);

                popServo.setPosition(0.4);
                topServo.setPosition(0.45);

                underglow.setPosition(0.333);
                break;
            case SHOOT:
                switch(shootState) {
                    case SET:
                        if ((veloError <= 10 && Math.abs(turretError) <= 4)){
                            shootState = ShootStates.FIRE;
                        }

                        intakeMotor.setPower(1);
                        popperMotor.setPower(0.8);

                        popServo.setPosition(0.21);
                        topServo.setPosition(0.45);

                        underglow.setPosition(0.277);
                        break;
                    case FIRE:
                        intakeMotor.setPower(1);
                        popperMotor.setPower(1);

                        topServo.setPosition(0.65);

                        if(gamepad1.leftBumperWasPressed()) popTimer.reset();

                        if(popTimer.milliseconds() < 100) popServo.setPosition(0.4);
                        else popServo.setPosition(0.21);

                        underglow.setPosition(0.444);
                        break;
                }
                break;
            case STOP:
                intakeMotor.setPower(0);
                popperMotor.setPower(0);

                topServo.setPosition(0.45);
                popServo.setPosition(0.4);
                break;
        }

        if(gamepad1.rightBumperWasPressed() && state != RobotStates.INTAKE) state = RobotStates.INTAKE;

        if(gamepad1.aWasPressed() && state != RobotStates.SHOOT){
            state = RobotStates.SHOOT;
            shootState = ShootStates.SET;
        }

        farLock = gamepad2.right_bumper;

        if(gamepad2.b){
            goalX = 140;
            limelight.pipelineSwitch(0);
            red = true;
        }else if(gamepad2.x){
            goalX = 4;
            limelight.pipelineSwitch(0);
            red = false;
        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);

        if(red) telemetry.addLine("RED");
        else telemetry.addLine("BLUE");

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.addData("state", state);
        telemetry.addData("Shoot-state", shootState);
    }

    double goalX = 4;
    double goalY = 140;

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

        double a = -3.3;
//        double a = -3.122222222222;
        double b = 0.353539;
        int turretTarget = (int)((targetAngle * a));

        int trueTarget = turretTarget;

        if (turretTarget < -260)
            turretTarget = -260;
        if (turretTarget > 260)
            turretTarget = 260;


        turretMotor.setTargetPosition(turretTarget);

        turretError = turretMotor.getCurrentPosition() - trueTarget;
    }

    int veloError = 0;

    private void velocityControl(){

        double distance = Math.sqrt((Math.pow((goalX - follower.getPose().getX()), 2) + Math.pow((goalY - follower.getPose().getY()), 2)));

        boolean close = distance <= 80;

        int target;
        if(close) target = (int)((6.91359 * distance) + 600.62726);
        else target = (int)((-0.0000668975 * Math.pow(distance,4)) + (0.0313581 * Math.pow(distance, 3)) + (-5.41008 * Math.pow(distance, 2)) + (411.39021 * distance) - 10410.3423);

//        -0.0000668975x^{4}+0.0313581x^{3}-5.41008x^{2}+411.39021x-10410.3423


        if(farLock && target < 1300) target = 1300;

        flywheelMotor.setVelocity(target);
        veloError = Math.abs((int)(flywheelMotor.getVelocity() - target));

        hoodControl(close);
    }

    private void hoodControl(boolean close){

        double hoodTarget;

        if(!close){
            hoodTarget = 0.52 * Math.pow(1.0005, veloError);

            if (hoodTarget > 0.57)
                hoodTarget = 0.57;
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

        Pose ftcStandard = PoseConverter.pose2DToPose(ftcPose2d, FTCCoordinates.INSTANCE);
        Pose current = ftcStandard.getAsCoordinateSystem(PedroCoordinates.INSTANCE);


        if(results.getBotposeAvgDist() != 0) follower.setPose(current);;
    }

    boolean lock = false;
    Pose lockPose = new Pose(0,0,0);
    private void drivetrain(){
        if(state != RobotStates.SHOOT){
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x * 0.8,
                    true // Robot Centric
            );
        }else{
                if(red) {
                    follower.setTeleOpDrive(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            (-0.00527008 * turretError),
                            false // field Centric
                    );
                }else{
                    follower.setTeleOpDrive(
                            gamepad1.left_stick_y,
                            gamepad1.left_stick_x,
                            (-0.00527008 * turretError),
                            false // field Centric
                    );
                }

        }

//        if(!lock){
//            lockPose = follower.getPose();
//            if(red) {
//                follower.setTeleOpDrive(
//                        -gamepad1.left_stick_y,
//                        -gamepad1.left_stick_x,
//                        (-0.00527008 * turretError),
//                        false // field Centric
//                );
//            }else{
//                follower.setTeleOpDrive(
//                        gamepad1.left_stick_y,
//                        gamepad1.left_stick_x,
//                        (-0.00527008 * turretError),
//                        false // field Centric
//                );
//            }
//            if(follower.getVelocity().getMagnitude() < 6 && Math.abs(follower.getAngularVelocity()) < 0.09 && Math.abs(turretError) < 5)
//                lock = true;
//        }else{
//            follower.holdPoint(lockPose,false);
//        }

//        if(lock) {
//            if ((Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.left_stick_y) + Math.abs(gamepad1.right_stick_x)) > 0.1) {
//                follower.startTeleopDrive();
//                lock = false;
//            }
//        }
    }
}
