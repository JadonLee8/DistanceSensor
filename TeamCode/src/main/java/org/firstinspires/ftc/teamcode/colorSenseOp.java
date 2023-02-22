package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class colorSenseOp extends OpMode {

    private ColorSensor cSensor;
    private DistanceSensor sensor;
    private DcMotorEx BLMotor;
    private DcMotorEx BRMotor;
    private DcMotorEx FLMotor;
    private DcMotorEx FRMotor;

    private boolean redMode = true;
    private boolean closed = false;

    @Override
    public void init() {
        cSensor = hardwareMap.get(ColorSensor.class, "cSensor");
        sensor = hardwareMap.get(DistanceSensor.class, "sensor");
        BLMotor = hardwareMap.get(DcMotorEx.class, "BLMotor");
        BRMotor = hardwareMap.get(DcMotorEx.class, "BRMotor");
        FLMotor = hardwareMap.get(DcMotorEx.class, "FLMotor");
        FRMotor = hardwareMap.get(DcMotorEx.class, "FRMotor");
    }

    @Override
    public void loop() {
        if (gamepad1.left_bumper){
            redMode = true;
        } else if (gamepad1.right_bumper){
            redMode = false;
        }

        if (gamepad1.a && redMode && (cSensor.red() > cSensor.blue() * 2.8) && (cSensor.red() < cSensor.blue() * 3.9)) {
            closed = true;
        } else if (gamepad1.a && !redMode && (cSensor.blue() > cSensor.red() * 2.8) && (cSensor.blue() < cSensor.red() * 5.0)){
            closed = true;
        } else if (gamepad1.b){
            closed = false;
        }

        if (redMode && (cSensor.red() > cSensor.blue() * 2.8) && (cSensor.red() < cSensor.blue() * 3.9) && !closed){
            gamepad1.rumble(200);
        } else if (!redMode && (cSensor.blue() > cSensor.red() * 3.9) && (cSensor.blue() < cSensor.red() * 5.2) && !closed){
            gamepad1.rumble(200);
        }

        telemetry.addData("Closed ? ", closed);
        telemetry.addData("Red Mode ? ", redMode);

        telemetry.addData("Blue Value: ", cSensor.blue());
        telemetry.addData("Red Value: ", cSensor.red());
        telemetry.update();
    }
}
