package org.firstinspires.ftc.teamcode.OpModes;


import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.LiftControlClass;

import java.util.List;

@TeleOp(name = "LED Program", group = "Competition")
@Disabled
public class LEDProgram extends LinearOpMode {
    private final static HWProfile robot = new HWProfile();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Ready to Run: ", "GOOD LUCK");
        telemetry.update();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        waitForStart();


        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        while (opModeIsActive()) {
            robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_PARTY_PALETTE);

        }   // end of while(opModeIsActive)
    }   // end of runOpMode()
}