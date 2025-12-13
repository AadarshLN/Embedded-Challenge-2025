 #include "mbed.h"
I2C i2c(PB_11, PB_10);  // I2C2: SDA = PB11, SCL = PB10
 

 //ignore this sometimes mac needs this to properly use printf
 // Create serial and bind it to printf
BufferedSerial serial_port(USBTX, USBRX, 115200);
FileHandle *mbed::mbed_override_console(int) {
    return &serial_port;  
}

 // LSM6DSL address (0x6A in datasheet, shifted left for 8-bit format)
 #define LSM6DSL_ADDR (0x6A << 1)  // Equals 0xD4

 // Please Refer to 48 and 49th pages in LSM6DSL datasheet 
 #define WHO_AM_I    0x0F  // ID register - should return 0x6A
 #define CTRL1_XL    0x10  // Accelerometer control register to configure range
 #define CTRL2_G     0x11  // Gyroscope control register to configure range
 #define OUTX_L_XL   0x28  // XL X-axis (low byte)
 #define OUTX_H_XL   0x29  // XL X-axis (high byte)
 #define OUTY_L_XL   0x2A  // XL Y-axis (low byte)
 #define OUTY_H_XL   0x2B  // XL Y-axis (high byte)
 #define OUTZ_L_XL   0x2C  // XL Z-axis (low byte)
 #define OUTZ_H_XL   0x2D  // XL Z-axis (high byte)
 #define OUTX_L_G    0x22  // Gyro X-axis (low byte)
 #define OUTX_H_G    0x23  // Gyro X-axis (high byte)
 #define OUTY_L_G    0x24  // Gyro Y-axis (low byte)
 #define OUTY_H_G    0x25  // Gyro Y-axis (high byte)
 #define OUTZ_L_G    0x26  // Gyro Z-axis (low byte)
 #define OUTZ_H_G    0x27  // Gyro Z-axis (high byte)
#define BUFFER_SIZE 64  // adjust size depending on RAM
#define FS 20.0f        // sampling frequency (Hz)

#define DYSK_MIN_FREQ 5.0f
#define DYSK_MAX_FREQ 7.0f

// Frequency band we care about
#define OSC_MIN_FREQ          3.0f
#define OSC_MAX_FREQ          5.0f

// How many windows to remember and how strict to be
#define FREQ_HISTORY_LEN      5       // last 5 windows
#define MIN_WINDOWS_IN_BAND   3       // at least 3 of them in band

#define MOVE_FREQ_MIN_HZ          1.0f   // "definitely moving" (step freq is usually ~1–2 Hz)
#define STILL_FREQ_MAX_HZ         0.35f  // treat below this as ~0 Hz
#define FOG_PREV_MOVE_WINDOWS     2      // need at least 2 recent moving windows
#define FOG_STILL_WINDOWS         1      // and 1 still windows to call FOG


float freq_history[FREQ_HISTORY_LEN];
bool  in_band_history[FREQ_HISTORY_LEN];
size_t history_idx    = 0;
size_t history_count  = 0;   // how many windows we've actually filled so far

float dysk_freq_history[FREQ_HISTORY_LEN];
bool  dysk_in_band_history[FREQ_HISTORY_LEN];
size_t dysk_history_idx    = 0;
size_t dysk_history_count  = 0;

#include "mbed.h"
#include "ble/BLE.h"
#include "ble/gatt/GattService.h"
#include "ble/gatt/GattCharacteristic.h"
#include "ble/Gap.h"
#include "ble/gap/AdvertisingDataBuilder.h"
#include "events/EventQueue.h"
#include <chrono>
#include <string.h>

// BufferedSerial serial_port(USBTX, USBRX, 115200);
// FileHandle *mbed::mbed_override_console(int) { return &serial_port; }

using namespace ble;
using namespace events;
using namespace std::chrono;

BLE &ble_interface = BLE::Instance();
EventQueue event_queue;
DigitalOut led(LED1);

const UUID TREMOR_SERVICE_UUID("A0E1B2C3-D4E5-F6A7-B8C9-D0E1F2A3B4C5");
const UUID TREMOR_TYPE_CHAR_UUID("A1E2B3C4-D5E6-F7A8-B9C0-D1E2F3A4B5C6");

const char* TREMOR_STRING     = "TREMOR";
const char* DYSKINESIA_STRING = "DYSKINESIA";
const char* FOG_STRING       = "FOG";

bool persistent_tremor = false;
bool persistent_dys = false;
bool fog_detected = false;

// ======================= CHANGED (1): buffer size + NO null terminator =======================
// BLE values are just bytes; to "broadcast ASCII", send only the ASCII bytes (no '\0').
// Longest string here is "DYSKINESIA" = 9 bytes.
#define MAX_TREMOR_STRING_LEN  9

uint8_t TREMORValue[MAX_TREMOR_STRING_LEN] = {0};
// ============================================================================================


// ======================= CHANGED (2): use variable-length GattCharacteristic =======================
// Your previous ReadOnlyArrayGattCharacteristic has a fixed-length view; switching to a
// variable-length characteristic avoids trailing garbage when strings have different lengths.
GattCharacteristic TREMORTypeCharacteristic(
    TREMOR_TYPE_CHAR_UUID,
    TREMORValue,
    0,                         // initial length (we'll write real length later)
    MAX_TREMOR_STRING_LEN,     // max length
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ |
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY
);
// ================================================================================================

GattCharacteristic *charTable[] = { &TREMORTypeCharacteristic };
GattService TREMOR_Service(TREMOR_SERVICE_UUID, charTable, 1);

bool isTremor = true;
Ticker notification_ticker;
bool device_connected = false;

// ======================= CHANGED (3): write ASCII bytes only (no null) =======================
static void set_ascii_value(const char* s) {
    size_t n = strlen(s);
    if (n > MAX_TREMOR_STRING_LEN) n = MAX_TREMOR_STRING_LEN;

    memset(TREMORValue, 0, sizeof(TREMORValue));          // clear old bytes
    memcpy(TREMORValue, s, n);                            // copy ASCII bytes

    ble_interface.gattServer().write(
        TREMORTypeCharacteristic.getValueHandle(),
        TREMORValue,
        n                                                // <-- NO +1, no null terminator
    );
}
// ============================================================================================

void send_TREMOR_notification() {
    if (!device_connected) {
        printf("No device connected, skipping notification\n");
        return;
    }

   // const char* msg = isTremor ? TREMOR_STRING : DYSKINESIA_STRING;
   const char* msg = persistent_tremor ? TREMOR_STRING : "Not tremor" ;
   
   // CHANGED: use persistent tremor flag
    set_ascii_value(msg);                                 // CHANGED: uses helper above

    printf("Sent notification (ASCII): %s\n", msg);

    led = !led;
    //isTremor = !isTremor;
}

void send_DYSKINESIA_notification() {
    if (!device_connected) {
        printf("No device connected, skipping notification\n");
        return;
    }

   // const char* msg = isTremor ? TREMOR_STRING : DYSKINESIA_STRING;
   const char* msg = persistent_dys ? DYSKINESIA_STRING : "Not dyskinesia" ;
   
   // CHANGED: use persistent tremor flag
    set_ascii_value(msg);                                 // CHANGED: uses helper above

    printf("Sent notification (ASCII): %s\n", msg);

    led = !led;
    //isTremor = !isTremor;
}

void send_FOG_notification() {
    if (!device_connected) {
        printf("No device connected, skipping notification\n");
        return;
    }

   // const char* msg = isTremor ? TREMOR_STRING : DYSKINESIA_STRING;
   const char* msg = fog_detected ? FOG_STRING : "Not fog" ;
   // CHANGED: use persistent tremor flag
    set_ascii_value(msg);                                 // CHANGED: uses helper above
    printf("Sent notification (ASCII): %s\n", msg);

    led = !led;
    //isTremor = !isTremor;
}


class ConnectionEventHandler : public ble::Gap::EventHandler {
public:
    virtual void onConnectionComplete(const ble::ConnectionCompleteEvent &event) {
        if (event.getStatus() == BLE_ERROR_NONE) {
            printf("Device connected!\n");
            device_connected = true;

            set_ascii_value(TREMOR_STRING);               // CHANGED: initialize with ASCII
            printf("hellohello");
            notification_ticker.attach([]() {
                event_queue.call(send_TREMOR_notification);
                event_queue.call_in(300ms,send_DYSKINESIA_notification);
                event_queue.call_in(600ms,send_FOG_notification);
            }, 1s);
        }
    }

    virtual void onDisconnectionComplete(const ble::DisconnectionCompleteEvent &event) {
        printf("Device disconnected!\n");
        device_connected = false;
        notification_ticker.detach();

        ble_interface.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);
        printf("Restarted advertising\n");
    }
};

ConnectionEventHandler connection_handler;

void on_ble_init_complete(BLE::InitializationCompleteCallbackContext *params) {
    if (params->error != BLE_ERROR_NONE) {
        printf("BLE initialization failed.\n");
        return;
    }

    // CHANGED: initialize characteristic with ASCII bytes (no '\0')
    set_ascii_value(TREMOR_STRING);

    ble_interface.gattServer().addService(TREMOR_Service);

    uint8_t adv_buffer[LEGACY_ADVERTISING_MAX_SIZE];
    AdvertisingDataBuilder adv_data(adv_buffer);

    adv_data.setFlags();
    adv_data.setName("TREMOR--Monitor");

    ble_interface.gap().setAdvertisingParameters(
        LEGACY_ADVERTISING_HANDLE,
        AdvertisingParameters(advertising_type_t::CONNECTABLE_UNDIRECTED, adv_interval_t(160))
    );

    ble_interface.gap().setAdvertisingPayload(
        LEGACY_ADVERTISING_HANDLE,
        adv_data.getAdvertisingData()
    );

    ble_interface.gap().setEventHandler(&connection_handler);
    ble_interface.gap().startAdvertising(LEGACY_ADVERTISING_HANDLE);

    printf("BLE advertising started as TREMOR--Monitor\n");
    printf("Waiting for device connection...\n");
}

void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context) {
    event_queue.call(callback(&ble_interface, &BLE::processEvents));
}


//bluetooth end


















 // Write a value to a register
 void write_register(uint8_t reg, uint8_t value) {
     char data[2] = {(char)reg, (char)value};
     i2c.write(LSM6DSL_ADDR, data, 2);
 }
 
 // Read a value from a register
 uint8_t read_register(uint8_t reg) {
     char data = reg;
     i2c.write(LSM6DSL_ADDR, &data, 1, true); // No stop
     i2c.read(LSM6DSL_ADDR, &data, 1);
     return (uint8_t)data;
 }
 
 // Read a 16-bit value (combines low and high byte registers)
 int16_t read_16bit_value(uint8_t low_reg, uint8_t high_reg) {
     // Read low byte
     char low_byte = read_register(low_reg);
     
     // Read high byte
     char high_byte = read_register(high_reg);
     
     // Combine the bytes (little-endian: low byte first)
     return (high_byte << 8) | low_byte;
 }
 

 void apply_hann_window(float *data, size_t N) {
    for (size_t i = 0; i < N; i++) {
        data[i] *= 0.5f * (1.0f - cosf(2.0f * M_PI * i / (N - 1)));
    }
}

bool update_and_check_persistent_oscillation(float freq_hz) {
    // 1) Is this window's dominant freq in the target band?
    bool in_band = (freq_hz >= OSC_MIN_FREQ && freq_hz <= OSC_MAX_FREQ);

    // 2) Store in circular history
    freq_history[history_idx]    = freq_hz;
    in_band_history[history_idx] = in_band;

    history_idx = (history_idx + 1) % FREQ_HISTORY_LEN;
    if (history_count < FREQ_HISTORY_LEN) {
        history_count++;
        return false;
    }

    // 4) Count how many recent windows were in-band
    size_t in_band_count = 0;
    for (size_t i = 0; i < history_count; i++) {
        if (in_band_history[i]) {
            in_band_count++;
        }
    }

    // 5) Persistent oscillation = enough windows in-band
    return (in_band_count >= MIN_WINDOWS_IN_BAND);
}


bool update_and_check_persistent_dysk(float freq_hz) {
    // 1) Is this window's dominant freq in the target band?
    bool in_band = (freq_hz >= DYSK_MIN_FREQ && freq_hz <= DYSK_MAX_FREQ);

    // 2) Store in circular history
    freq_history[history_idx]    = freq_hz;
    in_band_history[history_idx] = in_band;

    history_idx = (history_idx + 1) % FREQ_HISTORY_LEN;
    if (history_count < FREQ_HISTORY_LEN) {
        history_count++;
        return false;
    }

    
    // 4) Count how many recent windows were in-band
    size_t in_band_count = 0;
    for (size_t i = 0; i < history_count; i++) {
        if (in_band_history[i]) {
            in_band_count++;
        }
    }

    // 5) Persistent oscillation = enough windows in-band
    return (in_band_count >= MIN_WINDOWS_IN_BAND);
}



bool update_and_check_fog(float freq_hz)
{
    // Track how many recent windows were clearly moving vs clearly still
    static size_t recent_move_windows  = 0;
    static size_t recent_still_windows = 0;

    bool moving = (freq_hz >= MOVE_FREQ_MIN_HZ);
    bool still  = (freq_hz <= STILL_FREQ_MAX_HZ);  // freq==0 falls in here

    if (moving) {
        // We’re definitely moving in this window.
        if (recent_move_windows < FOG_PREV_MOVE_WINDOWS) {
            recent_move_windows++;
        }
        // movement breaks the "still" streak
        recent_still_windows = 0;
        printf("MOVING WINDOW DETECTED\n");
    }
    else if (still) {
        // We’re clearly still; only counts toward FOG if we had movement before.
        if (recent_move_windows >= FOG_PREV_MOVE_WINDOWS &&
            recent_still_windows < FOG_STILL_WINDOWS) {
            recent_still_windows++;
        }
        printf("STILL WINDOW DETECTED\n");
    }
    else {
        // In-between frequency: neither clearly moving nor still.
        recent_still_windows = 0;
    }

    bool fog_detected =
        (recent_move_windows >= FOG_PREV_MOVE_WINDOWS) &&
        (recent_still_windows >= FOG_STILL_WINDOWS);
    printf("Fog detected : %d\n", fog_detected);
    if (fog_detected) {
        // Reset so you can detect another FOG event later.
        recent_move_windows  = 0;
        recent_still_windows = 0;
    }
    

    return fog_detected;
}


size_t fft_find_dominant_freq(float *x, size_t N, float fs, float *freq_out) {
    // simple DFT
    float max_mag = 0.0f;
    size_t dominant_idx = 0;
    for (size_t k = 0; k < N/2; k++) {
        float re = 0.0f, im = 0.0f;
        for (size_t n = 0; n < N; n++) {
            float angle = 2.0f * M_PI * k * n / N;
            re += x[n] * cosf(angle);
            im -= x[n] * sinf(angle);
        }
        float mag = sqrtf(re*re + im*im);
        if (mag > max_mag) {
            max_mag = mag;
            dominant_idx = k;
        }
    }
    if (freq_out) {
        *freq_out = dominant_idx * fs / N;
    }
    return dominant_idx;
}



float estimate_frequency(int16_t *raw_buffer, size_t N) {
    float buffer[BUFFER_SIZE];

    // 1. Convert to float and detrend (remove mean)
    float mean = 0.0f;
    for (size_t i = 0; i < N; i++) mean += raw_buffer[i];
    mean /= N;

    float stddev = 0.0f;
    for (size_t i = 0; i < N; i++) {
        buffer[i] = (float)raw_buffer[i] - mean;
        stddev += buffer[i]*buffer[i];
    }
    stddev = sqrtf(stddev / N);

    const float STDDEV_MIN     = 1.0f;   // absolutely still
    const float STDDEV_WEAK    = 5.0f;   // noise-level motion

    if (stddev < STDDEV_MIN) {
        return 0.0f; // no movement at all
    }

    if (stddev < STDDEV_WEAK) {
        return 0.0f; // too weak for meaningful oscillation
    }
    
    // Normalize by standard deviation
    for (size_t i = 0; i < N; i++) {
        buffer[i] /= stddev;
    }

    // 2. Apply Hann window
    apply_hann_window(buffer, N);

    // 3. Compute DFT and find dominant frequency
    float freq = 0.0f;
    fft_find_dominant_freq(buffer, N, FS, &freq);

    return freq;
}

 int main() {
     // Setup I2C at 400kHz
     i2c.frequency(400000);
     
     // Check if sensor is connected
     uint8_t id = read_register(WHO_AM_I);
     
     if (id != 0x6A) {
         // printf("Error: LSM6DSL sensor not found!\r\n");
         while (1) { /* Stop here */ }
     }
     
    //  // Configure the accelerometer (104 Hz, ±2g range)
    write_register(CTRL1_XL, 0x24);  // 0010 0100 → ODR = 26 Hz, ±16g
     // 4. For ±16g range
    const float ACC_SENSITIVITY = 0.488f;  // mg/LSB
     
     // Configure the gyroscope (104 Hz, ±250 dps range)
     write_register(CTRL2_G, 0x40);
     int16_t acc_x_buffer[BUFFER_SIZE];
     int16_t acc_y_buffer[BUFFER_SIZE];
     int16_t acc_z_buffer[BUFFER_SIZE];
     size_t idx = 0;
     DigitalOut detect_led(LED1);   // LED shows persistent detection
 
     
    ble_interface.onEventsToProcess(schedule_ble_events);
    ble_interface.init(on_ble_init_complete);

    Thread ble_thread;
    ble_thread.start(callback(&event_queue, &EventQueue::dispatch_forever));

     // Main loop
     while (1) {

         // Read raw accelerometer values
         int16_t acc_x_raw = read_16bit_value(OUTX_L_XL, OUTX_H_XL);
         int16_t acc_y_raw = read_16bit_value(OUTY_L_XL, OUTY_H_XL);
         int16_t acc_z_raw = read_16bit_value(OUTZ_L_XL, OUTZ_H_XL);
         
         // take z axis
         acc_x_buffer[idx] = acc_x_raw;
         acc_y_buffer[idx] = acc_y_raw;
         acc_z_buffer[idx++] = acc_z_raw;

         if(idx >= BUFFER_SIZE) {
             float fz = estimate_frequency(acc_z_buffer, BUFFER_SIZE);
            persistent_dys = update_and_check_persistent_dysk(fz);
            printf("\nDyskinesia check:\n");
            printf("Window mag freq: %.2fHz |  persistent dyskinesia=%s\r\n",
                   fz,

                   persistent_dys ? "YES" : "NO");

            idx = 0; // reset buffer
            // 2) Update history & check persistence
            persistent_tremor = update_and_check_persistent_oscillation(fz);
            // 3) Print what’s going on
            printf("\n\nTremor check:\n");
            printf("Window freq: %.2f Hz | persistent(2.5-5.5Hz) = %s\r\n",
               fz,
               persistent_tremor ? "YES" : "NO");
            idx = 0;


            fog_detected = update_and_check_fog(fz);
            printf("FOG Detection check:\n");
            printf("\nWindow freq: %.2f Hz | Dyskinesia=%s | Tremor=%s | FOG=%s\r\n",
       fz,
       persistent_dys ? "YES" : "NO",
       persistent_tremor ? "YES" : "NO",
       fog_detected ? "YES" : "NO");

        }
       // Wait before next sample
         ThisThread::sleep_for(50ms);
    
        // printf("Starting BLE TREMOR Monitor...\n");


    //event_queue.dispatch_forever();
    

     }
    
 }
 