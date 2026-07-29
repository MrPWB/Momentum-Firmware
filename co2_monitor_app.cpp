#include <furi.h>
#include <furi_hal.h>
#include <furi_hal_light.h>
#include <gui/gui.h>
#include <gui/elements.h>
#include <storage/storage.h>
#include <stdlib.h>

#include <string>

#include "flipperscd30.h"
#include "csv_writer.h"

struct CO2Monitor {
    FuriMessageQueue* event_queue;

    SCD30Data data;
    ViewPort* view_port;
    Gui* gui;
    CsvWriter* csv;
    std::string last_data_ts;
};

static constexpr int CO2_MIN_LEVEL = 500;
static constexpr int CO2_MAX_LEVEL = 2500;
static constexpr int LED_MAX = 0xFF;

static void update_led(int co2_level) {
    int r = 0;
    int g = 0;

    if(co2_level < CO2_MIN_LEVEL) {
        g = LED_MAX;
    } else if(co2_level > CO2_MAX_LEVEL) {
        r = LED_MAX;
    } else {
        // Integer math: multiply first, then divide to maintain precision
        r = ((co2_level - CO2_MIN_LEVEL) * LED_MAX) / (CO2_MAX_LEVEL - CO2_MIN_LEVEL);
        g = LED_MAX - r;
    }

    furi_hal_light_set(LightRed, r / 2);
    furi_hal_light_set(LightGreen, g / 2);
    furi_hal_light_set(LightBlue, 0);
}

static void progress_bar(Canvas* canvas, int x, int y, int w, int progress, int max) {
    // Clamp progress between 0 and max
    progress = (progress < 0) ? 0 : (progress > max) ? max : progress;

    // Integer math: multiply first, then divide to maintain precision
    const int bar_width = ((w - 4) * progress) / max;

    canvas_draw_rframe(canvas, x, y, w, 7, 2);
    canvas_draw_rbox(canvas, x + 2, y + 2, bar_width, 3, 0);
}

static void draw_callback(Canvas* canvas, void* ctx) {
    CO2Monitor* context = static_cast<CO2Monitor*>(ctx);

    int co2 = static_cast<int>(context->data.co2_ppm);
    int temp = static_cast<int>(context->data.temperature);
    int hum = static_cast<int>(context->data.humidity);

    update_led(co2);

    int width = canvas_width(canvas);
    int center = width / 2;

    canvas_clear(canvas);

    // CO2 Display
    canvas_set_font(canvas, FontBigNumbers);
    canvas_draw_rframe(canvas, 5, 1, width - 10, 50, 10);
    canvas_draw_str_aligned(canvas, center, 5, AlignCenter, AlignTop, std::to_string(co2).c_str());
    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str_aligned(canvas, center, 22, AlignCenter, AlignTop, "ppm CO2");
    progress_bar(canvas, 10, 35, width - 20, co2, 3000);

    // Temp / humidity / pressure
    canvas_set_font(canvas, FontSecondary);
    char status_str[64];
    if(context->data.pressure_valid) {
        // Only one line fits, so the pressure replaces the calibration hint
        snprintf(
            status_str,
            sizeof(status_str),
            "%d C, %d %%, %d hPa",
            temp,
            hum,
            static_cast<int>(context->data.pressure_mbar));
    } else {
        snprintf(status_str, sizeof(status_str), "%d C, %d %% - Hold UP to calib.", temp, hum);
    }
    canvas_draw_str_aligned(canvas, 5, 55, AlignLeft, AlignTop, status_str);

    // Info
}

static void input_callback(InputEvent* input, void* ctx) {
    CO2Monitor* context = static_cast<CO2Monitor*>(ctx);
    furi_message_queue_put(context->event_queue, input, FuriWaitForever);
}

extern "C" int32_t co2_monitor_app(void* p) {
    UNUSED(p);

    CO2Monitor* co2_monitor = new CO2Monitor();
    co2_monitor->data = SCD30Data(); // Initialize with default values (zeros)

    co2_monitor->csv = new CsvWriter(APP_DATA_PATH("co2.csv"));

    co2_monitor->view_port = view_port_alloc();
    co2_monitor->gui = static_cast<Gui*>(furi_record_open("gui"));
    co2_monitor->event_queue = furi_message_queue_alloc(8, sizeof(InputEvent));
    view_port_draw_callback_set(co2_monitor->view_port, draw_callback, co2_monitor);
    view_port_input_callback_set(co2_monitor->view_port, input_callback, co2_monitor);

    gui_add_view_port(co2_monitor->gui, co2_monitor->view_port, GuiLayerFullscreen);

    FlipperSCD30WorkerThread scd30_worker(2000); // 2 second interval
    scd30_worker.start();

    bool running = true;
    InputEvent event;

    while(running) {
        FuriStatus status = furi_message_queue_get(co2_monitor->event_queue, &event, 100);
        if(status == FuriStatusOk) {
            if(event.type == InputTypePress && event.key == InputKeyBack) {
                running = false;
            } else if(event.type == InputTypeLong && event.key == InputKeyUp) {
                // Calibrate to 420ppm (average outside value)
                scd30_worker.calibrate_to(420);
            }
        }

        if(scd30_worker.has_data()) {
            SCD30Data new_data = scd30_worker.get_data();

            if(co2_monitor->last_data_ts != new_data.ts) {
                co2_monitor->data = new_data;
                co2_monitor->last_data_ts = new_data.ts;

                co2_monitor->csv->add_row({
                    new_data.ts,
                    std::to_string(new_data.co2_ppm),
                    std::to_string(new_data.temperature),
                    std::to_string(new_data.humidity),
                    new_data.pressure_valid ?
                        std::to_string(static_cast<int>(new_data.pressure_mbar)) :
                        "",
                });
            }
        }

        view_port_update(co2_monitor->view_port);
    }

    scd30_worker.stop();

    // Turn off LED
    furi_hal_light_set(LightRed, 0);
    furi_hal_light_set(LightGreen, 0);
    furi_hal_light_set(LightBlue, 0);

    gui_remove_view_port(co2_monitor->gui, co2_monitor->view_port);
    view_port_free(co2_monitor->view_port);
    furi_record_close("gui");

    delete co2_monitor->csv;
    delete co2_monitor;
    return 0;
}
