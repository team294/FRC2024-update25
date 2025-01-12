package frc.robot.utilities;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.Constants.BCRColor;

/** LED segment defines a range of LEDs and its current animation */
public class LEDSegment {
    private int index, count, frame;
    private boolean loop;
    private Color[][] animation;

    /**
     * Create the LED segment 
     * @param index start index
     * @param count the number of LEDs from start index
     * @param animation the current animation (animation[frame][LED ID])
     * @param loop whether the animation should loop (false = play once then stop)
     */
    public LEDSegment(int index, int count, Color[][] animation, boolean loop) {
        this.index = index;
        this.count = count;
        this.frame = 0;
        this.loop = loop;
        this.animation = animation;
    }

    /**
     * Create the LED segment that loops 
     * @param index start index
     * @param count the number of LEDs from start index
     * @param animation the current animation (animation[frame][LED ID])
     */
    public LEDSegment(int index, int count, Color[][] animation) {
        this(index, count, animation, true);
    }

    /**
     * Set/reset the animation of the segment (starts from the beginning)
     * @param animation the animation to play
     * @param loop whether the animation should loop
     */
    public void setAnimation(Color[][] animation, boolean loop) {
        this.frame = 0;
        this.animation = animation;
        this.loop = loop;
    }

    /**
     * Set/reset the pattern of the segment
     * @param pattern the pattern to display
     */
    public void setAnimation(Color[] pattern, boolean loop) {
        Color[][] animation = {pattern};
        setAnimation(animation, true);
    }

    /**
     * Set/reset the color of the segment
     * @param color the color to display
     */
    public void setAnimation(Color color) {
        Color[][] anim = {{color}};
        setAnimation(anim, true);
    }

    public Color[] getCurrentFrame() {
        if (isFinished()) return Constants.LEDConstants.Patterns.noPatternStatic;
        else return animation[frame];
    }

    /**
     * Return whether the animation has finished
     * @return whether the animation has finished
     */
    public boolean isFinished() {
        return frame >= animation.length;
    }

    /**
     * Move to the next frame
     * @return whether the animation has finished this exact frame (once)
     */
    public boolean advanceFrame() {
        frame++;
        if (isFinished()) {
            // Reached the end
            if (loop) frame = 0;
            else return frame == animation.length;
        }
        return false;
    }
}

