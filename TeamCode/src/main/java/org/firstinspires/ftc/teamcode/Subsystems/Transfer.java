package org.firstinspires.ftc.teamcode.Subsystems;

import java.util.*;
import org.firstinspires.ftc.vision.opencv.ColorRange;

public class Transfer {
    HashMap<Integer, String[]> spikeMarks = new HashMap<Integer, String[]>();

    {
        spikeMarks.put(1, new String[]{"P", "P", "G"});
        spikeMarks.put(2, new String[]{"P", "G", "P"});
        spikeMarks.put(3, new String[]{"G", "P", "P"});
    }
}
