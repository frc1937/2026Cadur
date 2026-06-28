package frc.lib.util;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import java.util.Arrays;

public class LEDStrip {
    private final int length;

    // Single int per LED: 0x00RRGGBB — no objects, no boxing
    private final int[] buffer;
    private final int[] scratchBuffer; // for scroll — swap instead of clone

    // -----------------------------------------------------------------------
    // Pre-computed cosine LUT for breathing/pulse (1024 steps, 0.0–1.0 range)
    // -----------------------------------------------------------------------
    private static final int LUT_SIZE = 1024;
    private static final int[] COS_LUT = new int[LUT_SIZE]; // scaled 0–255

    static {
        for (int i = 0; i < LUT_SIZE; i++) {
            double t = (1.0 - Math.cos(2.0 * Math.PI * i / LUT_SIZE)) * 0.5;
            COS_LUT[i] = (int) (t * 255);
        }
    }

    private int rngState;

    private int scrollTick = 0;
    private int scrollOffset = 0;
    private int breathTick = 0;
    private int flashTick = 0;
    private int flashColourIndex = 0;
    private int flashInterval = 0;
    private int cometTick = 0;

    public LEDStrip(int length) {
        this.length = length;
        this.buffer = new int[length];
        this.scratchBuffer = new int[length];
        this.rngState = 0xDEADBEEF ^ length;
    }

    public void writeToBuffer(AddressableLEDBuffer ledBuffer, int offset) {
        for (int i = 0; i < length; i++) {
            int rgb = buffer[i];
            ledBuffer.setRGB(offset + i,
                    (rgb >> 8) & 0xFF,
                    (rgb >> 16)  & 0xFF,
                    rgb        & 0xFF);
        }
    }

    public LEDStrip solidColour(int rgb) {
        Arrays.fill(buffer, rgb);
        return this;
    }

    /**
     * Smooth scrolling gradient. Colours packed as 0x00RRGGBB.
     *
     * scratchBuffer is only recomputed on even ticks — scroll() only
     * consumes it then, so odd-tick computation is pure waste.
     */
    public LEDStrip interpolated(int... colours) {
        if (colours.length == 0) return this;

        // Gate: only recompute when scroll() will actually consume the result.
        if ((scrollTick & 1) == 0) {
            int n = colours.length;
            for (int i = 0; i < length; i++) {
                int pos = i * n * 256 / length;    // 0 .. n*256
                int startIdx = pos >> 8;           // integer part
                int frac = pos & 0xFF;             // fractional part (0–255)
                int endIdx = (startIdx + 1) % n;

                scratchBuffer[i] = lerpRGB(colours[startIdx], colours[endIdx], frac);
            }
        }

        scroll(scratchBuffer);
        return this;
    }

    public LEDStrip rainbow() {
        return interpolated(
                0xFF0000, 0xFF8000, 0xFFFF00,
                0x00FF00, 0x0000FF, 0x4B0082, 0xEE82EE);
    }

    /**
     * flashInterval counter replaces `flashTick % 17` to avoid a
     * non-power-of-2 modulo on every periodic call.
     */
    public LEDStrip flashing(int... colours) {
        if (colours.length == 0) return this;

        if (++flashInterval >= 17) {
            flashInterval = 0;
            Arrays.fill(buffer, colours[flashColourIndex]);
            flashColourIndex = (flashColourIndex + 1) % colours.length;
        }

        flashTick++;
        return this;
    }

    public LEDStrip breathing(int from, int to) {
        int brightness = COS_LUT[breathTick & (LUT_SIZE - 1)];
        breathTick = (breathTick + 2) & (LUT_SIZE - 1);
        Arrays.fill(buffer, lerpRGB(from, to, brightness));

        return this;
    }

    public LEDStrip pulse(int colour, int speed) {
        int brightness = COS_LUT[breathTick & (LUT_SIZE - 1)];
        breathTick = (breathTick + speed) & (LUT_SIZE - 1);
        Arrays.fill(buffer, lerpRGB(0x000000, colour, brightness));

        return this;
    }

    /**
     * Sparkle — xorshift RNG, no allocation, density in 0–255 range (e.g. 8 = ~3%).
     */
    public LEDStrip sparkle(int base, int sparkleColour, int density255) {
        for (int i = 0; i < length; i++) {
            buffer[i] = (xorshift() & 0xFF) < density255 ? sparkleColour : base;
        }

        return this;
    }

    public LEDStrip strobe(int colour, int halfPeriod) {
        flashTick++;
        Arrays.fill(buffer, (flashTick % (halfPeriod * 2)) < halfPeriod ? colour : 0x000000);
        return this;
    }

    /**
     * Theatre chase — replaces per-pixel `i % spacing` with a fill + stride
     * write, eliminating `length` modulo operations per frame.
     */
    public LEDStrip theatreChase(int colour, int spacing) {
        int offset = (cometTick / 3) % spacing;
        cometTick++;

        Arrays.fill(buffer, 0x000000);
        for (int i = offset; i < length; i += spacing) {
            buffer[i] = colour;
        }

        return this;
    }

    /**
     * Comet with integer fade — tail brightness halved every step (bit shift).
     */
    public LEDStrip comet(int colour, int tailLength) {
        Arrays.fill(buffer, 0x000000);
        int head = (cometTick >> 1) % length;
        cometTick++;

        buffer[head] = colour;
        int faded = colour;
        for (int i = 1; i <= tailLength; i++) {
            int pos = (head - i + length) % length;
            faded = ((faded >> 1) & 0x7F7F7F);
            buffer[pos] = faded;
        }
        return this;
    }

    public LEDStrip wipeInward(int colourA, int colourB) {
        Arrays.fill(buffer, 0x000000);
        int mid = length >> 1;
        int progress = (cometTick >> 1) % (mid + 1);
        cometTick++;

        for (int i = 0; i < progress; i++) {
            buffer[i] = colourA;
            buffer[length - 1 - i] = colourB;
        }
        return this;
    }

    public LEDStrip outwardsPoints(int colour) {
        Arrays.fill(buffer, 0x000000);
        int quarter = length >> 2;
        int x = (scrollTick >> 2) % (quarter - 1);  // >> 2 replaces / 4
        scrollTick++;

        fillRange(quarter - 1 - x, quarter + 1 + x, colour);
        fillRange(quarter * 3 - x,  quarter * 3 + 2 + x, colour);
        return this;
    }

    /**
     * In-place scroll using two System.arraycopy calls instead of a
     * per-element loop — equivalent to a single JNI memmove each, ~10× faster.
     */
    private void scroll(int[] source) {
        if ((scrollTick & 1) == 0) {
            int rem = length - scrollOffset;
            System.arraycopy(source, scrollOffset, buffer, 0, rem);
            if (scrollOffset > 0) {
                System.arraycopy(source, 0, buffer, rem, scrollOffset);
            }
            scrollOffset = (scrollOffset + 1) % length;
        }
        scrollTick++;
    }

    /**
     * Fixed-point RGB lerp. frac in 0–255. No floats, no objects.
     * Operates on each channel packed in one int via masking.
     */
    private static int lerpRGB(int a, int b, int frac) {
        int invFrac = 256 - frac;
        int r = (((a >> 16) & 0xFF) * invFrac + ((b >> 16) & 0xFF) * frac) >> 8;
        int g = (((a >>  8) & 0xFF) * invFrac + ((b >>  8) & 0xFF) * frac) >> 8;
        int bl= (( a        & 0xFF) * invFrac + ( b        & 0xFF) * frac) >> 8;
        return (r << 16) | (g << 8) | bl;
    }

    private void fillRange(int from, int to, int colour) {
        int start = Math.max(0, from);
        int end   = Math.min(length, to);
        for (int i = start; i < end; i++) buffer[i] = colour;
    }

    /** Xorshift32 — period 2^32-1, one multiply-free, branch-free iteration. */
    private int xorshift() {
        rngState ^= rngState << 13;
        rngState ^= rngState >> 17;
        rngState ^= rngState << 5;
        return rngState;
    }

    public static int rgb(int r, int g, int b) {
        return (r << 16) | (g << 8) | b;
    }
}