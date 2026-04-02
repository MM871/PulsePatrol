#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

// --------------------------------------─
// PulsePatrol - FPGA Side (RISC-V / NIOS V soft core)
//
// This program runs on the RISC-V soft core inside the DE1-SoC FPGA.
// It does four main things:
//
//   1. RECEIVE - polls JP1 GPIO pins for incoming packets from the Arduino.
//      Each packet is a 16-bit frame: [distance (8 bits) | angle (8 bits)],
//      clocked in bit by bit on rising edges of the CLOCK line.
//
//   2. TRACK - maintains an array of up to 20 objects. Each detected reading
//      is matched against existing candidates. A candidate becomes a confirmed
//      object after 8 consecutive hits at roughly the same position. Confirmed
//      objects are frozen and assigned a numeric ID. Objects that stop being
//      detected expire after a few misses.
//
//   3. RENDER - every frame, clears the screen, redraws the radar arcs,
//      draws the live scanline at the current angle, and plots a red dot +
//      character ID label for every confirmed object.
//
//   4. INPUT - KEY0 hard-resets everything; KEY1 cycles the distance filter
//      through 25 cm → 18 cm → 10 cm, indicated by the red LEDs.
// --------------------------------------─


// -─ HEX display memory-mapped addresses ------------------─
// The DE1-SoC exposes the 7-segment displays as memory-mapped I/O.
// Writing a byte to these addresses lights the corresponding segments.
#define HEX3_0 ((volatile int *)0xFF200020)   // HEX displays 0–3
#define HEX5_4 ((volatile int *)0xFF200030)   // HEX displays 4–5

// Lookup table: index = decimal digit, value = 7-segment encoding
const unsigned char seg7[10] = {
    0x3F, // 0
    0x06, // 1
    0x5B, // 2
    0x4F, // 3
    0x66, // 4
    0x6D, // 5
    0x7D, // 6
    0x07, // 7
    0x7F, // 8
    0x6F  // 9
};


// -─ Object tracking parameters -----------------------
#define SIZE              20   // max number of tracked objects at once
#define ANGLE_TOL         10   // degrees - how close an angle must be to match
#define DIST_TOL           3   // cm      - how close a distance must be to match
#define CONFIRM_THRESHOLD  8   // consecutive hits needed to confirm an object
#define MISS_THRESHOLD     6   // misses before a confirmed object is removed
#define CANDIDATE_MISS_MAX 3   // misses before an unconfirmed candidate expires
#define EXCLUSION_ANGLE   20   // degrees - no new candidate within this of another
#define MIN_CREDIBLE_DIST  2   // cm - anything closer is sensor noise / echo


// -─ Object struct ------------------------------
// Each slot in objectArray represents one tracked thing (confirmed or candidate)
typedef struct {
    int  angle;        // frozen at confirmation - doesn't update after that
    int  distance;     // frozen at confirmation
    int  id;           // -1 = candidate (not displayed), >0 = confirmed (shown)
    bool empty;        // true = slot is free to use
    int  confirm_cnt;  // how many readings have been accumulated
    int  miss_cnt;     // how many consecutive scans missed this object
    int  consec_hits;  // consecutive hits - must reach CONFIRM_THRESHOLD
    int  angle_acc;    // running sum of angles (for centroid on confirmation)
    int  dist_acc;     // running sum of distances
} object;

object objectArray[SIZE];
int count = 0;   // global confirmed object ID counter - increments on each new one


// -─ Memory-mapped hardware addresses --------------------
#define JP1_BASE  ((volatile int *)0xFF200060)   // JP1 data register
#define JP1_DIR   ((volatile int *)0xFF200064)   // JP1 direction (0 = input)
#define JP1_MASK  ((volatile int *)0xFF200068)   // JP1 interrupt mask
#define KEY_BASE  ((volatile int *)0xFF200050)   // pushbutton data
#define KEY_EDGE  ((volatile int *)0xFF20005C)   // edge capture register
#define KEY_MASK  ((volatile int *)0xFF200058)   // pushbutton interrupt mask
#define LEDR_BASE ((volatile int *)0xFF200000)   // red LEDs


// -─ Character buffer ----------------------------─
// The DE1-SoC has a hardware character buffer at 0x09000000.
// It's an 80×60 grid of ASCII characters overlaid on the VGA output.
// We use it to display numeric ID labels next to confirmed objects.
#define CHAR_BUF_BASE ((volatile char *)0x09000000)


// -─ Filter state ------------------------------─
volatile bool clear_flag     = false;   // set by KEY0 ISR
const int FILTER_RANGES[3]   = {25, 18, 10};
volatile int filter_index    = 0;
volatile int active_max_dist = 25;      // current distance ceiling in cm
volatile bool filter_changed = false;   // set by KEY1 ISR


// -─ VGA display constants --------------------------
// The radar is drawn as a semicircle centered at (RADAR_X, RADAR_Y)
#define RADAR_X  160   // center X of radar (horizontal midpoint of 320px screen)
#define RADAR_Y  200   // center Y - pushed toward bottom so arcs have room
#define RADAR_R  150   // radius of the outermost radar arc in pixels
#define MAX_DIST  25   // max distance in cm - maps to RADAR_R pixels

#define BLACK  0x0000
#define GREEN  0x07E0
#define RED    0xF800
#define WHITE  0xFFFF


// -─ Double-buffered pixel buffers ----------------------
// We draw into the back buffer while the front buffer is displayed,
// then swap on vsync - this eliminates screen tearing.
volatile int pixel_buffer_start;
short int Buffer1[240][512];
short int Buffer2[240][512];

int current_angle    = 90;   // last received angle - starts pointing straight up
int current_distance = 0;    // last received distance


// -─ Duplicate packet suppression ----------------------
// If the Arduino sends the exact same (distance, angle) twice in a row
// (can happen at slow sweep speeds), we ignore the duplicate so it doesn't
// count as two separate hits toward confirmation.
int last_poll_dist  = -1;
int last_poll_angle = -1;


// ════════════════════════════════════════════════════════════════════════════
// Character buffer helpers
// ════════════════════════════════════════════════════════════════════════════

// Write a single ASCII character to character cell (cx, cy)
void char_put(int cx, int cy, char c) {
    if (cx < 0 || cx >= 80 || cy < 0 || cy >= 60) return;
    *(CHAR_BUF_BASE + (cy << 7) + cx) = c;
    // Address formula: base + row*128 + col
    // (128 = next power of 2 above 80, as required by the hardware)
}

// Write a null-terminated string starting at (cx, cy)
void char_print(int cx, int cy, const char *s) {
    while (*s) char_put(cx++, cy, *s++);
}

// Blank the entire character buffer (fill with spaces)
void clear_char_buf(void) {
    for (int cy = 0; cy < 60; cy++)
        for (int cx = 0; cx < 80; cx++)
            char_put(cx, cy, ' ');
}

// Convert a positive integer ID to a decimal string (no stdlib needed)
void id_to_str(int id, char *buf) {
    if (id <= 0) { buf[0] = '0'; buf[1] = '\0'; return; }
    int n = 0;
    char tmp[8];
    while (id > 0) { tmp[n++] = '0' + (id % 10); id /= 10; }
    // tmp is reversed - flip it into buf
    for (int i = 0; i < n; i++) buf[i] = tmp[n - 1 - i];
    buf[n] = '\0';
}

// Given a pixel position (px, py) of a detected object, figure out where
// to place the character label so it doesn't go off screen.
// Prefer slightly to the right; fall back to the left if too close to edge.
void get_label_char_pos(int px, int py, int *cx_out, int *cy_out) {
    int lx = px + 3, ly = py;
    if (lx > 316) lx = px - 7;   // too close to right edge - go left instead
    *cx_out = lx / 4;             // convert pixel → character column (4px per cell)
    *cy_out = ly / 4;             // convert pixel → character row
    // clamp to valid character grid bounds
    if (*cx_out <  0) *cx_out = 0;
    if (*cx_out > 77) *cx_out = 77;
    if (*cy_out <  0) *cy_out = 0;
    if (*cy_out > 59) *cy_out = 59;
}


// ════════════════════════════════════════════════════════════════════════════
// Graphics helpers
// ════════════════════════════════════════════════════════════════════════════

// Write one pixel to the current back buffer at (x, y)
void plot_pixel(int x, int y, short int color) {
    if (x < 0 || x >= 320 || y < 0 || y >= 240) return;
    volatile short int *addr =
        (volatile short int *)(pixel_buffer_start + (y << 10) + (x << 1));
    *addr = color;
    // Address formula: base + row*1024 + col*2
    // (1024 because each row in the buffer is 512 shorts = 1024 bytes)
}

// Fill the entire screen with black
void clear_screen(void) {
    for (int y = 0; y < 240; y++)
        for (int x = 0; x < 320; x++)
            plot_pixel(x, y, BLACK);
}

// Draw the radar arc grid - three concentric semicircles at 1/3, 2/3, and
// full radius, sampled every 2 degrees for a dotted appearance
void draw_radar_arcs(void) {
    for (int deg = 0; deg <= 180; deg += 2) {
        float rad = deg * 3.14159f / 180.0f;
        for (int r = RADAR_R / 3; r <= RADAR_R; r += RADAR_R / 3) {
            int x = RADAR_X + (int)(r * cosf(rad));
            int y = RADAR_Y - (int)(r * sinf(rad));
            plot_pixel(x, y, GREEN);
        }
    }
}

// Draw a filled square block centered at (x, y) - used for object dots
void plot_pixel_block(int x, int y, int size, short int color) {
    for (int i = -size; i <= size; i++)
        for (int j = -size; j <= size; j++)
            plot_pixel(x + j, y + i, color);
}

void swap(int *a, int *b) { int t = *a; *a = *b; *b = t; }

// Bresenham's line algorithm - draws a straight line between two points
void draw_line(int x0, int y0, int x1, int y1, short int color) {
    bool steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep)   { swap(&x0, &y0); swap(&x1, &y1); }
    if (x0 > x1) { swap(&x0, &x1); swap(&y0, &y1); }
    int dx = x1 - x0, dy = abs(y1 - y0), err = -(dx / 2);
    int y = y0, ys = (y0 < y1) ? 1 : -1;
    for (int x = x0; x <= x1; x++) {
        steep ? plot_pixel(y, x, color) : plot_pixel(x, y, color);
        err += dy;
        if (err > 0) { y += ys; err -= dx; }
    }
}

// Block until the VGA controller has finished displaying the current frame,
// then swap front and back buffers
void wait_for_vsync(void) {
    volatile int *p = (int *)0xFF203020;
    *p = 1;                    // write 1 to request a buffer swap
    while (*(p + 3) & 0x01);  // wait until the S bit clears (swap done)
}

// Display angle on HEX3–HEX1 and distance on HEX5–HEX4
void show_hex(int angle, int distance) {
    int ah = angle / 100, at = (angle / 10) % 10, ao = angle % 10;
    int dt = distance / 10, dlo = distance % 10;
    *HEX3_0 = ((seg7[ah] << 16) | (seg7[at] << 8) | seg7[ao]);
    *HEX5_4 = ((seg7[dt] << 8) | seg7[dlo]);
}


// ════════════════════════════════════════════════════════════════════════════
// poll_packet - blocking GPIO receiver
//
// Waits for the Arduino to send a complete 16-bit packet over the 3-wire
// bit-bang protocol (SYNC/CLOCK/DATA on JP1 bits 2/1/0).
//
// Protocol timing (from the Arduino side):
//   SYNC goes HIGH → 20 ms gap → 16 clock pulses (20 µs high, 20 µs low each)
//   → SYNC goes LOW
//
// Returns true and fills *dist_out / *ang_out if a valid packet arrived.
// Returns false on timeout or if the values are out of range.
// ════════════════════════════════════════════════════════════════════════════
bool poll_packet(int *dist_out, int *ang_out) {
    const int TIMEOUT = 10000000;
    int t = 0;

    // Disable global interrupts while we're bit-banging - we can't afford
    // to be interrupted mid-packet or we'll lose bits
    asm volatile("csrc mstatus, %0" ::"r"(0x8));

    // - Wait for SYNC to go HIGH (start of frame) ------------─
    while ((*JP1_BASE & 0x4) == 0) {
        if (++t > TIMEOUT) {
            asm volatile("csrs mstatus, %0" ::"r"(0x8));
            return false;
        }
    }

    // - Wait for CLOCK to go LOW (Arduino's idle state before first bit) -
    t = 0;
    while ((*JP1_BASE & 0x2) != 0) {
        if ((*JP1_BASE & 0x4) == 0) {   // SYNC dropped early - abort
            asm volatile("csrs mstatus, %0" ::"r"(0x8));
            return false;
        }
        if (++t > TIMEOUT) {
            asm volatile("csrs mstatus, %0" ::"r"(0x8));
            return false;
        }
    }

    // - Shift in 16 bits, sampling DATA on each rising CLOCK edge ----─
    int shift = 0, bits = 0, last_clk = 0;
    t = 0;
    while (bits < 16) {
        int snap = *JP1_BASE;   // read all JP1 bits atomically

        if ((snap & 0x4) == 0) {   // SYNC dropped - packet was cut short
            asm volatile("csrs mstatus, %0" ::"r"(0x8));
            return false;
        }

        int clk = snap & 0x2;
        if (clk && !last_clk) {
            // Rising edge detected - latch the current DATA bit
            shift = (shift << 1) | (snap & 0x1);
            bits++;
        }
        last_clk = clk;

        if (++t > TIMEOUT) {
            asm volatile("csrs mstatus, %0" ::"r"(0x8));
            return false;
        }
    }

    // - Wait for SYNC to go LOW (end of frame) --------------
    t = 0;
    while ((*JP1_BASE & 0x4) != 0) {
        if (++t > TIMEOUT) {
            asm volatile("csrs mstatus, %0" ::"r"(0x8));
            return false;
        }
    }

    // Re-enable global interrupts now that we have the full packet
    asm volatile("csrs mstatus, %0" ::"r"(0x8));

    // - Unpack: high byte = distance, low byte = angle ----------
    int d = (shift >> 8) & 0xFF;
    int a = shift & 0xFF;

    // Validate - reject obviously bad readings before they enter the tracker
    if (a > 180)  return false;   // angle out of physical range
    if (d == 0 || d == 255) return false;   // 0 = no return, 255 = too far sentinel
    if (d < MIN_CREDIBLE_DIST) return false; // too close = echo / cross-talk noise

    *dist_out = d;
    *ang_out  = a;
    return true;
}


// Object tracking
//
// The tracker maintains up to SIZE slots. Each slot is either:
//   - Empty (available)
//   - A candidate (id == -1): accumulating hits, not yet displayed
//   - Confirmed (id > 0): frozen position, displayed with ID label
//
// A candidate becomes confirmed after CONFIRM_THRESHOLD *consecutive* hits
// (not just cumulative - scattered noise hits don't promote it).
// Position is frozen as the centroid of all accumulated readings.
//
// Objects expire when they've been missed too many times:
//   - Candidates expire after CANDIDATE_MISS_MAX misses
//   - Confirmed objects expire after MISS_THRESHOLD misses
// Misses are only counted when a reading is within ANGLE_TOL of the object's

void arrayInitialisation(void) {
    for (int i = 0; i < SIZE; i++) {
        objectArray[i].angle       = 0;
        objectArray[i].distance    = 0;
        objectArray[i].id          = -1;
        objectArray[i].empty       = true;
        objectArray[i].confirm_cnt = 0;
        objectArray[i].miss_cnt    = 0;
        objectArray[i].consec_hits = 0;
        objectArray[i].angle_acc   = 0;
        objectArray[i].dist_acc    = 0;
    }
}

static int iabs(int x) { return x < 0 ? -x : x; }

// Get the reference angle for matching: use the frozen value for confirmed
// objects, or the running average for candidates
static int slot_ref_angle(const object *o) {
    if (o->id != -1)        return o->angle;
    if (o->confirm_cnt > 0) return o->angle_acc / o->confirm_cnt;
    return o->angle_acc;
}
static int slot_ref_dist(const object *o) {
    if (o->id != -1)        return o->distance;
    if (o->confirm_cnt > 0) return o->dist_acc / o->confirm_cnt;
    return o->dist_acc;
}

// Reset a slot back to empty
static void slot_clear(object *o) {
    o->empty       = true;
    o->id          = -1;
    o->confirm_cnt = 0;
    o->miss_cnt    = 0;
    o->consec_hits = 0;
    o->angle_acc   = 0;
    o->dist_acc    = 0;
}

void objectCheck(int distance, int angle) {
    // Is this reading within our active filter range and credible?
    bool present = (distance >= MIN_CREDIBLE_DIST &&
                    distance < active_max_dist &&
                    distance != 255);

    // - 1. Find the closest matching existing slot ------------─
    int best = -1, best_dd = 9999;
    for (int i = 0; i < SIZE; i++) {
        if (objectArray[i].empty) continue;
        int ref_a = slot_ref_angle(&objectArray[i]);
        int ref_d = slot_ref_dist(&objectArray[i]);
        int da = iabs(angle    - ref_a);
        int dd = iabs(distance - ref_d);
        if (da <= ANGLE_TOL && dd <= DIST_TOL && dd < best_dd) {
            best_dd = dd;
            best    = i;
        }
    }

    // - 2. Hit - reading matches an existing slot -------------
    if (present && best >= 0) {
        object *o = &objectArray[best];
        o->miss_cnt = 0;   // reset miss counter - it's still alive

        if (o->id == -1) {
            // Still a candidate - accumulate this reading
            o->angle_acc += angle;
            o->dist_acc  += distance;
            o->confirm_cnt++;
            o->consec_hits++;

            // Promote to confirmed once we have enough consecutive hits
            if (o->consec_hits >= CONFIRM_THRESHOLD) {
                // Freeze position as centroid of all accumulated readings
                o->angle    = o->angle_acc / o->confirm_cnt;
                o->distance = o->dist_acc  / o->confirm_cnt;
                o->id       = ++count;   // assign the next available ID
            }
        }
        // Confirmed objects: position is intentionally frozen - don't update
        return;
    }

    // - 3. Miss - nothing at this angle, increment nearby miss counters --
    if (!present) {
        for (int i = 0; i < SIZE; i++) {
            if (objectArray[i].empty) continue;
            int ref_a = slot_ref_angle(&objectArray[i]);
            if (iabs(angle - ref_a) <= ANGLE_TOL) {
                objectArray[i].miss_cnt++;
                objectArray[i].consec_hits = 0;   // break the consecutive streak

                // Expire the slot if it's missed too many times
                int limit = (objectArray[i].id == -1)
                            ? CANDIDATE_MISS_MAX
                            : MISS_THRESHOLD;
                if (objectArray[i].miss_cnt >= limit) {
                    slot_clear(&objectArray[i]);
                }
            }
        }
        return;
    }

    // - 4. New candidate - only if no existing slot is nearby ------─
    // The exclusion zone prevents two candidates from tracking the same object
    for (int i = 0; i < SIZE; i++) {
        if (objectArray[i].empty) continue;
        int ref_a = slot_ref_angle(&objectArray[i]);
        if (iabs(angle - ref_a) <= EXCLUSION_ANGLE) return;
    }

    // Find a free slot and initialise a new candidate
    for (int i = 0; i < SIZE; i++) {
        if (!objectArray[i].empty) continue;
        objectArray[i].empty       = false;
        objectArray[i].id          = -1;   // candidate - not shown yet
        objectArray[i].confirm_cnt = 1;
        objectArray[i].miss_cnt    = 0;
        objectArray[i].consec_hits = 1;
        objectArray[i].angle_acc   = angle;
        objectArray[i].dist_acc    = distance;
        break;
    }
}


// ════════════════════════════════════════════════════════════════════════════
// Interrupt handlers
//
// KEY0 (bit 0): hard reset - clears all objects and resets filter to 25 cm
// KEY1 (bit 1): cycles distance filter 25 → 18 → 10 → 25 cm
//               LEDs show how many cm the active filter allows
// ════════════════════════════════════════════════════════════════════════════
void key_isr(void) {
    int pressed = *KEY_EDGE;
    *KEY_EDGE = pressed;   // clear the edge capture register

    if (pressed & 0x1) {
        // KEY0 pressed - signal main loop to do a full reset
        clear_flag = true;
    }
    if (pressed & 0x2) {
        // KEY1 pressed - advance to next filter range
        filter_index    = (filter_index + 1) % 3;
        active_max_dist = FILTER_RANGES[filter_index];
        filter_changed  = true;
        // Immediately update HEX display to show new range
        *HEX5_4 = ((seg7[active_max_dist / 10] << 8) | seg7[active_max_dist % 10]);
    }
}

// The RISC-V trap handler - called on any interrupt or exception.
// We only care about interrupt cause 18 (GPIO/button interrupts on this core).
void __attribute__((interrupt)) trap_handler(void) {
    int mcause;
    asm volatile("csrr %0, mcause" : "=r"(mcause));
    if ((mcause & 0x80000000) && (mcause & 0x1F) == 18) key_isr();
}

void key_init(void) {
    *KEY_EDGE = 0xF;   // clear any pending edge captures
    *KEY_MASK = 0x3;   // enable interrupts for KEY0 and KEY1
}

void interrupt_init(void) {
    asm volatile("csrw mtvec, %0" ::"r"(trap_handler));   // set trap vector
    asm volatile("csrw mie,   %0" ::"r"(1 << 18));        // enable cause 18
    asm volatile("csrs mstatus, %0" ::"r"(0x8));          // enable global interrupts
}


// ════════════════════════════════════════════════════════════════════════════
// main
// ════════════════════════════════════════════════════════════════════════════
int main(void) {
    volatile int *pixel_ctrl_ptr = (int *)0xFF203020;

    // JP1 all inputs (we only read from it), no interrupt mask
    *JP1_DIR  = 0x0;
    *JP1_MASK = 0x0;

    key_init();
    interrupt_init();
    arrayInitialisation();

    // - Set up double buffering ----------------------
    // Point the back buffer register at Buffer1, wait for vsync to swap it
    // to front, then Buffer2 becomes the new back buffer we draw into
    *(pixel_ctrl_ptr + 1) = (int)Buffer1;
    wait_for_vsync();
    pixel_buffer_start = *pixel_ctrl_ptr;
    clear_screen();
    *(pixel_ctrl_ptr + 1) = (int)Buffer2;
    clear_char_buf();

    // - Main loop ----------------------------─
    while (true) {
        // Always draw into the back buffer
        pixel_buffer_start = *(pixel_ctrl_ptr + 1);

        // - Handle KEY0 hard reset --------------------─
        if (clear_flag) {
            arrayInitialisation();
            count           = 0;
            filter_index    = 0;
            active_max_dist = 25;
            filter_changed  = false;
            clear_flag      = false;
            *LEDR_BASE      = 0;
            clear_char_buf();
            last_poll_dist  = -1;
            last_poll_angle = -1;
        }

        // - Handle KEY1 filter change -------------------
        // Clear tracked objects so stale detections outside the new range
        // don't linger on screen
        if (filter_changed) {
            arrayInitialisation();
            count          = 0;
            filter_changed = false;
            clear_char_buf();
            last_poll_dist  = -1;
            last_poll_angle = -1;
        }

        // - Update LED bar to show active filter range ----------─
        if      (active_max_dist == 25) *LEDR_BASE = 0b0000000111;
        else if (active_max_dist == 18) *LEDR_BASE = 0b0000011111;
        else if (active_max_dist == 10) *LEDR_BASE = 0b1111111111;
        else                            *LEDR_BASE = 0;

        // - Draw background ------------------------
        clear_screen();
        draw_radar_arcs();

        // - Try to receive one packet from the Arduino ----------─
        int d, a;
        if (poll_packet(&d, &a)) {

            // Suppress duplicate packets - same (d, a) twice in a row
            bool duplicate = (d == last_poll_dist && a == last_poll_angle);
            last_poll_dist  = d;
            last_poll_angle = a;

            current_distance = d;
            current_angle    = a;
            show_hex(current_angle, current_distance);

            if (!duplicate) {
                objectCheck(current_distance, current_angle);
            }
        }

        // - Draw confirmed objects --------------------─
        clear_char_buf();
        for (int i = 0; i < SIZE; i++) {
            if (objectArray[i].empty || objectArray[i].id == -1) continue;

            // Convert polar (angle, distance) → screen (x, y)
            float rad = objectArray[i].angle * 3.14159265f / 180.0f;
            int rp = objectArray[i].distance * RADAR_R / MAX_DIST;
            if (rp > RADAR_R) rp = RADAR_R;
            int ox = RADAR_X + (int)(rp * cosf(rad));
            int oy = RADAR_Y - (int)(rp * sinf(rad));

            // Draw a green line from center to object, then a red dot on top
            draw_line(RADAR_X, RADAR_Y, ox, oy, GREEN);
            plot_pixel_block(ox, oy, 2, RED);

            // Write the numeric ID label next to the dot
            int cx, cy;
            char label[8];
            get_label_char_pos(ox, oy, &cx, &cy);
            id_to_str(objectArray[i].id, label);
            char_print(cx, cy, label);
        }

        // - Draw scanline -------------------------
        // White line from center to edge at the current sweep angle —
        // always drawn at full length regardless of distance reading
        {
            float rad = current_angle * 3.14159265f / 180.0f;
            int rx = RADAR_X + (int)(RADAR_R * cosf(rad));
            int ry = RADAR_Y - (int)(RADAR_R * sinf(rad));
            draw_line(RADAR_X, RADAR_Y, rx, ry, WHITE);

            // Also draw a red dot at the actual measured distance on the scanline
            if (current_distance > 0 && current_distance != 255) {
                int rp = current_distance * RADAR_R / MAX_DIST;
                if (rp > RADAR_R) rp = RADAR_R;
                int ex = RADAR_X + (int)(rp * cosf(rad));
                int ey = RADAR_Y - (int)(rp * sinf(rad));
                plot_pixel_block(ex, ey, 2, RED);
            }
        }

        // - Swap buffers on vsync ---------------------
        *(pixel_ctrl_ptr + 1) = (int)pixel_buffer_start;
        wait_for_vsync();
    }

    return 0;
}