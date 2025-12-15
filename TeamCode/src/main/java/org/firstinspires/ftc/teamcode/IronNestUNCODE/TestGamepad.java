package org.firstinspires.ftc.teamcode.IronNestUNCODE;

import com.bylazar.gamepad.GamepadManager;
import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import kotlin.jvm.internal.Intrinsics;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.jetbrains.annotations.NotNull;

@TeleOp(name = "Test Gamepad", group = "Dev")
public class TestGamepad extends OpMode {
    @NotNull
    private final GamepadManager g1;
    @NotNull
    private final GamepadManager g2;
    @NotNull
    private final TelemetryManager panelsTelemetry;

    public TestGamepad() {
        this.g1 = PanelsGamepad.INSTANCE.getFirstManager();
        this.g2 = PanelsGamepad.INSTANCE.getSecondManager();
        this.panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @NotNull
    public final GamepadManager getG1() {
        return this.g1;
    }

    @NotNull
    public final GamepadManager getG2() {
        return this.g2;
    }

    @NotNull
    public final TelemetryManager getPanelsTelemetry() {
        return this.panelsTelemetry;
    }

    public void init() {
    }

    public void loop() {
        GamepadManager var10000 = this.g1;
        Gamepad var10001 = this.gamepad1;
        Gamepad g1 = var10000.asCombinedFTCGamepad(var10001);
        var10000 = this.g2;
        var10001 = this.gamepad2;
        Gamepad g2 = var10000.asCombinedFTCGamepad(var10001);
        TelemetryManager var28 = this.panelsTelemetry;
        String[] var3 = new String[]{"==== Buttons ===="};
        var28.debug(var3);
        var28 = this.panelsTelemetry;
        var3 = new String[]{"A: " + g1.a};
        var28.debug(var3);
        var28 = this.panelsTelemetry;
        var3 = new String[]{"B: " + g1.b};
        var28.debug(var3);
        var28 = this.panelsTelemetry;
        var3 = new String[]{"X: " + g1.x};
        var28.debug(var3);
        var28 = this.panelsTelemetry;
        var3 = new String[]{"Y: " + g1.y};
        var28.debug(var3);
        var28 = this.panelsTelemetry;
        var3 = new String[]{"DPad Up: " + g1.dpad_up};
        var28.debug(var3);
        var28 = this.panelsTelemetry;
        var3 = new String[]{"DPad Down: " + g1.dpad_down};
        var28.debug(var3);
        var28 = this.panelsTelemetry;
        var3 = new String[]{"DPad Left: " + g1.dpad_left};
        var28.debug(var3);
        var28 = this.panelsTelemetry;
        var3 = new String[]{"DPad Right: " + g1.dpad_right};
        var28.debug(var3);
        var28 = this.panelsTelemetry;
        var3 = new String[]{"Left Bumper: " + g1.left_bumper};
        var28.debug(var3);
        var28 = this.panelsTelemetry;
        var3 = new String[]{"Right Bumper: " + g1.right_bumper};
        var28.debug(var3);
        var28 = this.panelsTelemetry;
        var3 = new String[]{"Left Trigger: " + g1.left_trigger};
        var28.debug(var3);
        var28 = this.panelsTelemetry;
        var3 = new String[]{"Right Trigger: " + g1.right_trigger};
        var28.debug(var3);
        var28 = this.panelsTelemetry;
        var3 = new String[]{"Start / Options: " + g1.options};
        var28.debug(var3);
        var28 = this.panelsTelemetry;
        var3 = new String[]{"Back / Share: " + g1.back};
        var28.debug(var3);
        var28 = this.panelsTelemetry;
        var3 = new String[]{"Guide / PS: " + g1.guide};
        var28.debug(var3);
        var28 = this.panelsTelemetry;
        var3 = new String[]{"Touchpad: " + g1.touchpad};
        var28.debug(var3);
        var28 = this.panelsTelemetry;
        var3 = new String[]{"Left Stick Button: " + g1.left_stick_button};
        var28.debug(var3);
        var28 = this.panelsTelemetry;
        var3 = new String[]{"Right Stick Button: " + g1.right_stick_button};
        var28.debug(var3);
        var28 = this.panelsTelemetry;
        var3 = new String[]{"==== Sticks ===="};
        var28.debug(var3);
        var28 = this.panelsTelemetry;
        var3 = new String[]{"Left Stick X: " + g1.left_stick_x};
        var28.debug(var3);
        var28 = this.panelsTelemetry;
        var3 = new String[]{"Left Stick Y: " + g1.left_stick_y};
        var28.debug(var3);
        var28 = this.panelsTelemetry;
        var3 = new String[]{"Right Stick X: " + g1.right_stick_x};
        var28.debug(var3);
        var28 = this.panelsTelemetry;
        var3 = new String[]{"Right Stick Y: " + g1.right_stick_y};
        var28.debug(var3);
        var28 = this.panelsTelemetry;
        Telemetry var54 = this.telemetry;
        Intrinsics.checkNotNullExpressionValue(var54, "telemetry");
        var28.update(var54);
    }
}
