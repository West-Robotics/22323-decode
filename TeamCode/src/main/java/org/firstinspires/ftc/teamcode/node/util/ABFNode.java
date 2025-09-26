package org.firstinspires.ftc.teamcode.node.util;

import org.firstinspires.ftc.teamcode.node.Node;

import java.util.HashMap;
import java.util.List;

// Alpha-Beta Filter Node
public class ABFNode extends Node {
    private double x, v, r, a, b = 0.0;
    String tx, tv, s;
    public ABFNode(String title_x, String title_v, String source, double alpha, double beta) {
        tx = title_x;
        tv = title_v;
        s = source;
        a = alpha;
        b = beta;
    }
    @Override
    public void init() {
        subscriptions.add(s);
        subscriptions.add("d_t");
    }
    @Override
    public void loop() {
        x = x + (double) data.get("d_t")*v;
        r = (double) data.get(s) - x;
        x = x + a*r;
        v = v + (b/(double) data.get("d_t"))*r;
    }
    public HashMap<String, Object> publish() {
        HashMap<String, Object> message = new HashMap<String, Object>();
        message.put(tx, x);
        message.put(tv, v);
        return message;
    }
}