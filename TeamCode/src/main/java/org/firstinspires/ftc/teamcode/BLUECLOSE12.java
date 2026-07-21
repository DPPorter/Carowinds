package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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


@Autonomous(name = "BLUE-CLOSE_12", group = "Auto")
@Configurable // Panels
public class BLUECLOSE12 extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower;
    public Follower followerAim; // Pedro Pathing follower instance

    private enum pathStates {SHOOT_PRE, PICK_SPIKE1, SHOOT_SPIKE1, PICK_SPIKE2, SHOOT_SPIKE2, PICK_GATE1, SHOOT_GATE1, PARK, DONE}
    pathStates pathState; // Current autonomous path state (state machine)

    private Paths paths;// Paths defined in the Paths class


    private enum States {INTAKE, REST, SET, FIRE, STOP}
    States state = States.INTAKE;


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

    PIDFCoefficients pidVariables = new PIDFCoefficients(250, 0, 0, 17.7, MotorControlAlgorithm.PIDF);

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(15.204, 114.400, Math.toRadians(0)));

        paths = new Paths(follower);// Build paths

        pathState = pathStates.SHOOT_PRE;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);


        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
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
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine
        robotControl();

        telemetry.addData("VelocityError", veloError);
        telemetry.addData("turretError", turretError);
        telemetry.addData("targetVelo", targetVelo);
        telemetry.addData("State", state);
        telemetry.addData("beam", beamState);

        telemetry.addLine();

        telemetry.addData("outTime", outTimer.milliseconds());
        telemetry.addData("thereTime", thereTimer.milliseconds());
    }




    public static class Paths {
        public PathChain preload;
        public PathChain pickspike1;
        public PathChain shootspike1;
        public PathChain pickspike2;
        public PathChain shootspike2;
        public PathChain pickgate1;
        public PathChain shootgate1;
        public PathChain park;

        public Paths(Follower follower) {
            preload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.204, 114.400),

                                    new Pose(44.519, 86.271)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(190))

                    .build();

            pickspike1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(44.519, 86.271),
                                    new Pose(14.669, 84.809),
                                    new Pose(23.293, 78.837),
                                    new Pose(16.431, 74.619)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(190), Math.toRadians(180))

                    .build();

            shootspike1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(16.431, 74.619),

                                    new Pose(60.249, 71.785)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(200))

                    .build();

            pickspike2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(60.249, 71.785),
                                    new Pose(39.936, 58.304),
                                    new Pose(20.840, 59.575)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            shootspike2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(20.840, 59.575),

                                    new Pose(61.133, 70.017)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(175), Math.toRadians(180))

                    .build();

            pickgate1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(61.133, 70.017),
                                    new Pose(21.315, 51.602),
                                    new Pose(11.122, 60.669)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            shootgate1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(11.122, 60.669),

                                    new Pose(59.740, 71.309)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))

                    .build();

            park = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(59.740, 71.309),

                                    new Pose(42.110, 71.309)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();
        }
    }



    ElapsedTime driveTime = new ElapsedTime();
    public pathStates autonomousPathUpdate() {
        switch(pathState){
            case SHOOT_PRE:
                follower.followPath(paths.preload);
                pathState = pathStates.PICK_SPIKE1;
                state = States.REST;
                break;
            case PICK_SPIKE1:
                if(!follower.isBusy()){
                    if(state == States.REST)
                        state = States.SET;
                }
                if(state == States.INTAKE){
                    follower.followPath(paths.pickspike1);
                    pathState = pathStates.SHOOT_SPIKE1;
                }
                break;
            case SHOOT_SPIKE1:
                if(!follower.isBusy()){
                    follower.followPath(paths.shootspike1);
                    pathState = pathStates.PICK_SPIKE2;
                    state = States.REST;
                }
                break;
            case PICK_SPIKE2:
                if(!follower.isBusy()){
                    if(state == States.REST)
                        state = States.SET;
                }
                if(state == States.INTAKE) {
                    follower.followPath(paths.pickspike2);
                    pathState = pathStates.SHOOT_SPIKE2;
                    driveTime.reset();
                }
                break;
            case SHOOT_SPIKE2:
                if(!follower.isBusy() || driveTime.seconds() > 3){
                    follower.followPath(paths.shootspike2);
                    pathState = pathStates.PICK_GATE1;
                    state = States.REST;
                }
                break;
            case PICK_GATE1:
                if(!follower.isBusy()){
                    if(state == States.REST)
                        state = States.SET;
                }
                if(state == States.INTAKE){
                    follower.followPath(paths.pickgate1);
                    pathState = pathStates.SHOOT_GATE1;
                    driveTime.reset();
                }
                break;
            case SHOOT_GATE1:
                if(!follower.isBusy() || driveTime.seconds() > 3){
                    follower.followPath(paths.shootgate1);
                    pathState = pathStates.PARK;
                    state = States.REST;
                }
                break;
            case PARK:
                if(!follower.isBusy()){
                    if(state == States.REST)
                        state = States.SET;
                }

                if(state == States.INTAKE){
                    follower.followPath(paths.park);
                    pathState = pathStates.DONE;
                    state = States.STOP;
                }
                break;
            case DONE:

                break;
        }
        // Event markers will automatically trigger at their positions
        // Make sure to register NamedCommands in your RobotContainer
        return pathState;
    }




    private void llReset(){
        LLResult results = limelight.getLatestResult();

        Pose2D ftcPose2d = new Pose2D(DistanceUnit.INCH, (results.getBotpose().getPosition().x * 39.3701), (results.getBotpose().getPosition().y * 39.3701), AngleUnit.RADIANS, AngleUnit.normalizeRadians(results.getBotpose().getOrientation().getYaw(AngleUnit.RADIANS)));

        Pose ftcStandard = PoseConverter.pose2DToPose(ftcPose2d, FTCCoordinates.INSTANCE);
        Pose current = ftcStandard.getAsCoordinateSystem(PedroCoordinates.INSTANCE);


        if(results.getBotposeAvgDist() != 0) followerAim.setPose(current);;
    }

    int goalX = 4;
    int goalY = 140;

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

        if (turretTarget < -260)
            turretTarget = -260;
        if (turretTarget > 260)
            turretTarget = 260;


        turretMotor.setTargetPosition(turretTarget);

        turretError = turretMotor.getCurrentPosition() - trueTarget;
    }

    int veloError = 0;

    int targetVelo = 0;

    private void velocityControl(){

        double distance = Math.sqrt((Math.pow((goalX - follower.getPose().getX()), 2) + Math.pow((goalY - follower.getPose().getY()), 2)));

        boolean close = distance <= 80;

        int target;
        if(close) target = (int)((6.91359 * distance) + 600.62726);
        else target = (int)((-0.0000668975 * Math.pow(distance,4)) + (0.0313581 * Math.pow(distance, 3)) + (-5.41008 * Math.pow(distance, 2)) + (411.39021 * distance) - 10410.3423);

        if(target < 1410) target = 1410;

        targetVelo = target;

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

    ElapsedTime popTimer = new ElapsedTime();

    private void robotControl(){
        beamState = beamCheck();

//        llReset();
        velocityControl();
        turretControl();

        switch(state){
            case INTAKE:
                intakeMotor.setPower(1);
                popperMotor.setPower(1);

                popServo.setPosition(0.4);
                topServo.setPosition(0.45);
                break;
            case REST:
                intakeMotor.setPower(0.7);
                popperMotor.setPower(0.9);

                popServo.setPosition(0.4);
                topServo.setPosition(0.45);
                break;
            case SET:
                if ((veloError <= 10 && Math.abs(turretError) <= 4))
                    state = States.FIRE;

                intakeMotor.setPower(1);
                popperMotor.setPower(0.8);

                popServo.setPosition(0.21);
                topServo.setPosition(0.45);

                thereTimer.reset();
                outTimer.reset();
                waitShot.reset();
                break;
            case FIRE:
                intakeMotor.setPower(1);
                popperMotor.setPower(1);

                topServo.setPosition(0.65);

                if(beamState == 1) popTimer.reset();
                else if(beamState == 0  && waitShot.milliseconds() > 1500) state = States.INTAKE;

                if(popTimer.milliseconds() < 150) popServo.setPosition(0.4);
                else popServo.setPosition(0.21);

                break;
            case STOP:
                intakeMotor.setPower(0);
                popperMotor.setPower(0);

                flywheelMotor.setVelocity(0);

                topServo.setPosition(0.45);
                popServo.setPosition(0.4);
                break;
        }
    }


    ElapsedTime outTimer = new ElapsedTime();
    ElapsedTime thereTimer = new ElapsedTime();

    ElapsedTime waitShot = new ElapsedTime();

    int beamState = 0;

    // 0: nothing
    // 1: pop
    // 2: pause
    private int beamCheck(){
        boolean objThere = false;

        int count = 0;
        for(int i = 9; i >= 0; i --)
            if(outtakeBeam.getState()) count ++;
        if(count >= 4) objThere = true;

        if(!objThere) thereTimer.reset();

        if(objThere || veloError > 40) outTimer.reset();

        if(thereTimer.milliseconds() > 150)
            return 1;
        else if(outTimer.milliseconds() > 550)
            return 0;
        else
            return 2;
    }

}


