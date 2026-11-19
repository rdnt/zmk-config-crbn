/*
 * keygrid_status_screen.c
 *
 * ZMK status screen for the nice!view (160x68 Sharp memory LCD), landscape.
 * Draws a 4 x 12 grid of 8x8 px keys:
 *   - released key -> 50 % dither (pattern in keygrid_dither(), i/y based)
 *   - pressed key  -> solid
 *
 * Key state comes from raw zmk_position_state_changed events, i.e. physical
 * key positions. Layers, &mo/&lt/&mt, etc. do not matter: every keydown and
 * keyup on a position is mirrored on the display.
 *
 * Target: ZMK main (Zephyr 4.1 / LVGL 9).
 * SPDX-License-Identifier: MIT
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include <lvgl.h>

#include <zmk/display.h>
#include <zmk/display/status_screen.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>

/* ------------------------------------------------------------------------ */
/* Options                                                                   */
/* ------------------------------------------------------------------------ */

/* 0 = dark keys on light background (stock nice!view look)
 * 1 = light keys on dark background
 * Driven by CONFIG_ZMK_KEYGRID_INVERTED; replace with 0/1 to hardcode. */
#define KEYGRID_INVERTED IS_ENABLED(CONFIG_ZMK_KEYGRID_INVERTED)

/* 1 = reverse landscape (image rotated 180 degrees), 0 = normal landscape.
 * Driven by CONFIG_ZMK_KEYGRID_ROTATE_180; replace with 0/1 to hardcode. */
#define KEYGRID_ROTATE_180 IS_ENABLED(CONFIG_ZMK_KEYGRID_ROTATE_180)

#define KEYGRID_ROWS 4
#define KEYGRID_COLS 12
#define KEYGRID_KEY_SIZE 8  /* px, keys are square                         */
#define KEYGRID_KEY_GAP 3   /* px between neighbouring keys (2 or 3)       */
#define KEYGRID_SPLIT_GAP 0 /* extra px between column 6 and 7 (0 = none)  */

/* ------------------------------------------------------------------------ */
/* Geometry                                                                  */
/* ------------------------------------------------------------------------ */

#define DISP_W 160
#define DISP_H 68

#define KEYGRID_CELLS (KEYGRID_ROWS * KEYGRID_COLS)
#define KEY_PITCH (KEYGRID_KEY_SIZE + KEYGRID_KEY_GAP)

#define GRID_W                                                                                     \
    (KEYGRID_COLS * KEYGRID_KEY_SIZE + (KEYGRID_COLS - 1) * KEYGRID_KEY_GAP + KEYGRID_SPLIT_GAP)
#define GRID_H (KEYGRID_ROWS * KEYGRID_KEY_SIZE + (KEYGRID_ROWS - 1) * KEYGRID_KEY_GAP)

/* Centered on the panel */
#define GRID_X ((DISP_W - GRID_W) / 2)
#define GRID_Y ((DISP_H - GRID_H) / 2)

BUILD_ASSERT(GRID_W <= DISP_W && GRID_H <= DISP_H, "key grid does not fit on the nice!view");
BUILD_ASSERT(KEYGRID_CELLS <= 64, "pressed state is kept in a 64-bit mask");

/* ------------------------------------------------------------------------ */
/* Key position -> grid cell                                                 */
/* ------------------------------------------------------------------------ */

/*
 * Index = ZMK key position (the order of bindings in your keymap / matrix
 * transform). Value = grid cell (row * KEYGRID_COLS + col), or -1 to ignore.
 * Positions past the end of the table are ignored.
 * Default: plain row-major 4x12 board.
 */
static const int8_t keygrid_position_map[] = {
    0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, /* row 0 */
    12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, /* row 1 */
    24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, /* row 2 */
    36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, /* row 3 */
};

static int position_to_cell(uint32_t position) {
    if (position >= ARRAY_SIZE(keygrid_position_map)) {
        return -1;
    }
    int cell = keygrid_position_map[position];
    return (cell >= 0 && cell < KEYGRID_CELLS) ? cell : -1;
}

/* ------------------------------------------------------------------------ */
/* Dithering                                                                 */
/* ------------------------------------------------------------------------ */

/*
 * Pattern for a RELEASED key. Called once per pixel of the key:
 *   i = x inside the key, 0 .. KEYGRID_KEY_SIZE-1 (left -> right)
 *   y = y inside the key, 0 .. KEYGRID_KEY_SIZE-1 (top  -> bottom)
 * Coordinates are as you see them (rotation is applied afterwards).
 * Return true to draw ink on that pixel.
 *
 * Alternatives:
 *   ((i + y) & 1) == 0                      50 % checkerboard (default)
 *   (i & 1) == 0                            50 % vertical lines
 *   (y & 1) == 0                            50 % horizontal lines
 *   ((i | y) & 1) == 0                      25 % dots
 *   ((i + y) % 3) == 0                      33 % diagonals
 *   i == 0 || y == 0 ||
 *   i == KEYGRID_KEY_SIZE - 1 ||
 *   y == KEYGRID_KEY_SIZE - 1               outline only
 */
static bool keygrid_dither(int i, int y) { return ((i + y) & 1) == 0; }

/* ------------------------------------------------------------------------ */
/* Pixel buffer (L8 canvas, same format the stock nice!view widget uses)     */
/* ------------------------------------------------------------------------ */

#define CANVAS_CF LV_COLOR_FORMAT_L8
#define CANVAS_BUF_SIZE                                                                            \
    LV_CANVAS_BUF_SIZE(DISP_W, DISP_H, LV_COLOR_FORMAT_GET_BPP(CANVAS_CF), LV_DRAW_BUF_STRIDE_ALIGN)

/* L8: 0x00 = black, 0xFF = white */
#define LUM_INK (KEYGRID_INVERTED ? 0xFF : 0x00)
#define LUM_PAPER (KEYGRID_INVERTED ? 0x00 : 0xFF)

static uint8_t keygrid_canvas_buf[CANVAS_BUF_SIZE];

/* Display thread only */
static lv_obj_t *keygrid_canvas;
static uint8_t *px_data;
static uint32_t px_stride;
static uint64_t drawn_cells;

/* Written from event context, read from display thread */
static ATOMIC_DEFINE(pressed_cells, KEYGRID_CELLS);

static inline void put_px(int x, int y, bool ink) {
    if (x < 0 || y < 0 || x >= DISP_W || y >= DISP_H) {
        return;
    }
    if (KEYGRID_ROTATE_180) {
        x = DISP_W - 1 - x;
        y = DISP_H - 1 - y;
    }
    px_data[(uint32_t)y * px_stride + (uint32_t)x] = ink ? LUM_INK : LUM_PAPER;
}

static inline int key_x(int col) {
    return GRID_X + col * KEY_PITCH + (col >= KEYGRID_COLS / 2 ? KEYGRID_SPLIT_GAP : 0);
}

static inline int key_y(int row) { return GRID_Y + row * KEY_PITCH; }

static void draw_key(int cell, bool pressed) {
    const int x0 = key_x(cell % KEYGRID_COLS);
    const int y0 = key_y(cell / KEYGRID_COLS);

    for (int y = 0; y < KEYGRID_KEY_SIZE; y++) {
        for (int i = 0; i < KEYGRID_KEY_SIZE; i++) {
            bool ink = pressed ? true : keygrid_dither(i, y);
            put_px(x0 + i, y0 + y, ink);
        }
    }
}

/* Invalidate only the key's rectangle so the LCD refreshes just those lines */
static void invalidate_key(int cell) {
    int x1 = key_x(cell % KEYGRID_COLS);
    int y1 = key_y(cell / KEYGRID_COLS);
    int x2 = x1 + KEYGRID_KEY_SIZE - 1;
    int y2 = y1 + KEYGRID_KEY_SIZE - 1;

    if (KEYGRID_ROTATE_180) {
        int tx = x1, ty = y1;
        x1 = DISP_W - 1 - x2;
        x2 = DISP_W - 1 - tx;
        y1 = DISP_H - 1 - y2;
        y2 = DISP_H - 1 - ty;
    }

    lv_area_t obj;
    lv_obj_get_coords(keygrid_canvas, &obj);
    lv_area_t area = {obj.x1 + x1, obj.y1 + y1, obj.x1 + x2, obj.y1 + y2};
    lv_obj_invalidate_area(keygrid_canvas, &area);
}

static uint64_t pressed_snapshot(void) {
    uint64_t mask = 0;
    for (int c = 0; c < KEYGRID_CELLS; c++) {
        if (atomic_test_bit(pressed_cells, c)) {
            mask |= BIT64(c);
        }
    }
    return mask;
}

static void draw_all(uint64_t cells) {
    for (int y = 0; y < DISP_H; y++) {
        memset(px_data + (uint32_t)y * px_stride, LUM_PAPER, DISP_W);
    }
    for (int c = 0; c < KEYGRID_CELLS; c++) {
        draw_key(c, (cells & BIT64(c)) != 0);
    }
    lv_obj_invalidate(keygrid_canvas);
}

/* ------------------------------------------------------------------------ */
/* Display-thread update                                                     */
/* ------------------------------------------------------------------------ */

static void keygrid_update_work_cb(struct k_work *work) {
    ARG_UNUSED(work);

    if (keygrid_canvas == NULL) {
        return;
    }

    const uint64_t now = pressed_snapshot();
    const uint64_t changed = now ^ drawn_cells;

    for (int c = 0; c < KEYGRID_CELLS; c++) {
        if (changed & BIT64(c)) {
            draw_key(c, (now & BIT64(c)) != 0);
            invalidate_key(c);
        }
    }
    drawn_cells = now;
}

static K_WORK_DEFINE(keygrid_update_work, keygrid_update_work_cb);

/* ------------------------------------------------------------------------ */
/* Event listener: raw key positions, independent of layers/behaviors        */
/* ------------------------------------------------------------------------ */

static int keygrid_position_listener(const zmk_event_t *eh) {
    const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    const int cell = position_to_cell(ev->position);
    if (cell >= 0) {
        if (ev->state) {
            atomic_set_bit(pressed_cells, cell);
        } else {
            atomic_clear_bit(pressed_cells, cell);
        }

        if (zmk_display_is_initialized()) {
            k_work_submit_to_queue(zmk_display_work_q(), &keygrid_update_work);
        }
    }

    /* Never consume the event - the keymap still has to see it */
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(keygrid, keygrid_position_listener);
ZMK_SUBSCRIPTION(keygrid, zmk_position_state_changed);

/* ------------------------------------------------------------------------ */
/* Screen entry point (called by ZMK on the display work queue)              */
/* ------------------------------------------------------------------------ */

lv_obj_t *zmk_display_status_screen(void) {
    lv_obj_t *screen = lv_obj_create(NULL);
    lv_obj_remove_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(screen, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(screen, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(screen, KEYGRID_INVERTED ? lv_color_black() : lv_color_white(),
                              LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);

    keygrid_canvas = lv_canvas_create(screen);
    lv_canvas_set_buffer(keygrid_canvas, keygrid_canvas_buf, DISP_W, DISP_H, CANVAS_CF);
    lv_obj_set_pos(keygrid_canvas, 0, 0);

    /* Use LVGL's (possibly aligned) data pointer and stride */
    lv_draw_buf_t *db = lv_canvas_get_draw_buf(keygrid_canvas);
    px_data = db->data;
    px_stride = db->header.stride;

    drawn_cells = pressed_snapshot();
    draw_all(drawn_cells);

    return screen;
}
