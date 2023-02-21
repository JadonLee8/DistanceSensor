package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

@TeleOp
public class MainOp extends OpMode {

    private DistanceSensor sensor;
    private DcMotorEx BLMotor;
    private DcMotorEx BRMotor;
    private DcMotorEx FLMotor;
    private DcMotorEx FRMotor;

    private double joystickY;
    private double joystickX;

    private double kp = .5;

    private int ticksPerRev = 28;
    private int wheelDiameterMM = 90;
    private double mmToInchConversion = 25.4;
    private double wheelDiameterIn = wheelDiameterMM / mmToInchConversion;
    private double gearRatio = 13.1;
    private double ticksPerInch = ((Math.PI * wheelDiameterIn) / gearRatio) / ticksPerRev;

    private int closePosIn = 2;
    private int farPosIn = 10;

    @Override
    public void init() {
        sensor = hardwareMap.get(DistanceSensor.class, "sensor");
        BLMotor = hardwareMap.get(DcMotorEx.class, "BLMotor");
        BRMotor = hardwareMap.get(DcMotorEx.class, "BRMotor");
        FLMotor = hardwareMap.get(DcMotorEx.class, "FLMotor");
        FRMotor = hardwareMap.get(DcMotorEx.class, "FRMotor");

        BLMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FLMotor.setDirection(DcMotorSimple.Direction.REVERSE);

//        BLMotor.setPositionPIDFCoefficients(kp);
//        BRMotor.setPositionPIDFCoefficients(kp);
//        FLMotor.setPositionPIDFCoefficients(kp);
//        FRMotor.setPositionPIDFCoefficients(kp);

        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        BLMotor.setTargetPositionTolerance((int)(.1 * ticksPerInch));
//        BRMotor.setTargetPositionTolerance((int)(.1 * ticksPerInch));
//        FLMotor.setTargetPositionTolerance((int)(.1 * ticksPerInch));
//        FRMotor.setTargetPositionTolerance((int)(.1 * ticksPerInch));
    }

    @Override
    public void loop() {
        if (gamepad1.a || gamepad1.b) {
            if (gamepad1.a && (Math.abs(closePosIn - sensor.getDistance(DistanceUnit.INCH)) > .5)) {
//            int startingPos = BLMotor.getCurrentPosition();
//            setTargetPos((int)(startingPos + (sensor.getDistance(DistanceUnit.INCH) - closePosIn) * ticksPerInch));
                if (gamepad1.a && closePosIn > sensor.getDistance(DistanceUnit.INCH)) {
                    telemetry.addData("Sensor Value: ", sensor.getDistance(DistanceUnit.INCH) - 2.1);
                    setPowers(-1);
                } else {
                    setPowers(0);
                }

                if (gamepad1.a && closePosIn < sensor.getDistance(DistanceUnit.INCH)) {
                    telemetry.addData("Sensor Value: ", sensor.getDistance(DistanceUnit.INCH) - 2.1);

                    setPowers(-.1);
                } else {
                    setPowers(0);
                }

            } else if (gamepad1.b && (Math.abs(farPosIn - sensor.getDistance(DistanceUnit.INCH)) > .5)) {
                if (gamepad1.b && farPosIn > sensor.getDistance(DistanceUnit.INCH)) {
                    telemetry.addData("Sensor Value: ", sensor.getDistance(DistanceUnit.INCH) - 2.1);

                    setPowers(.1);
                } else {
                    setPowers(0);
                }

                if (gamepad1.b && (farPosIn < sensor.getDistance(DistanceUnit.INCH))) {
                    telemetry.addData("Sensor Value: ", sensor.getDistance(DistanceUnit.INCH) - 2.1);

                    setPowers(-.1);
                } else {
                    setPowers(0);
                }
            }
        } else {

            joystickX = gamepad1.left_stick_x;
            joystickY = gamepad1.left_stick_y;

            drive(joystickY, joystickX);
        }

        telemetry.addData("Sensor Value: ", sensor.getDistance(DistanceUnit.INCH) - 2.1);
//        telemetry.addData("BLMotor Encoder Pos: ", BLMotor.getCurrentPosition());
//        telemetry.addData("Target Position: ", BLMotor.getTargetPosition());
//        telemetry.update();
    }



//    private void setTargetPos(int pos){
//        BLMotor.setTargetPosition(pos);
//        BRMotor.setTargetPosition(pos);
//        FLMotor.setTargetPosition(pos);
//        FRMotor.setTargetPosition(pos);
//
//        BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }

    private void setPowers(double power){
        BLMotor.setPower(power);
        BRMotor.setPower(power);
        FLMotor.setPower(power);
        FRMotor.setPower(power);
    }

    private void drive(double yIn, double xIn){
        double BLPower = yIn + xIn;
        double BRPower = yIn - xIn;
        double FLPower = yIn + xIn;
        double FRPower = yIn - xIn;

        double[] powers = {Math.abs(BLPower), Math.abs(BRPower), Math.abs(FLPower), Math.abs(FRPower)};
        Arrays.sort(powers);
        if (powers[3] > 1){
            BLPower /= powers[3];
            BRPower /= powers[3];
            FLPower /= powers[3];
            FRPower /= powers[3];
        }

        telemetry.addData("BLPower: ", BLPower);
        telemetry.addData("BRPower: ", BRPower);
        telemetry.addData("FLPower: ", FLPower);
        telemetry.addData("FRPower: ", FRPower);

        BLMotor.setPower(BLPower);
        BRMotor.setPower(BRPower);
        FLMotor.setPower(FLPower);
        FRMotor.setPower(FRPower);
    }
}
