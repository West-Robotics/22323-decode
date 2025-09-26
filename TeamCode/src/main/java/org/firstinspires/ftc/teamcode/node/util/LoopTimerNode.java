package org.firstinspires.ftc.teamcode.node.util;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.node.Node;
import java.util.HashMap;

public class LoopTimerNode extends Node {
    ElapsedTime timer;

    @Override
    public void init() {
        timer = new ElapsedTime();
    }
    @Override
    public void loop() {
        timer.reset();
    }
    public HashMap<String, Object> publish() {
        HashMap<String, Object> message = new HashMap<String, Object>();
        message.put("d_t", timer.milliseconds());
        return message;
    }
}