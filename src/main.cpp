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


#define DYSK_MIN_FREQ 4.5f
#define DYSK_MAX_FREQ 7.5f

// Frequency band we care about
#define OSC_MIN_FREQ          2.5f
#define OSC_MAX_FREQ          5.5f

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

    // // 3) If we don't have enough windows yet, we can't make a strong statement
    // if (history_count < FREQ_HISTORY_LEN) {
    //        // "not yet sure"
    // }

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


// bool update_and_check_persistent_dysk(bool in_band, float combined_freq_for_logging) {
//     dysk_freq_history[dysk_history_idx]    = combined_freq_for_logging;
//     dysk_in_band_history[dysk_history_idx] = in_band;

//     dysk_history_idx = (dysk_history_idx + 1) % FREQ_HISTORY_LEN;
//     if (dysk_history_count < FREQ_HISTORY_LEN) dysk_history_count++;

//     if (dysk_history_count < FREQ_HISTORY_LEN) return false;

//     size_t in_count = 0;
//     for (size_t i = 0; i < dysk_history_count; i++) {
//         if (dysk_in_band_history[i]) in_count++;
//     }
//     return (in_count >= MIN_WINDOWS_IN_BAND);
// }
// Update history based on a single magnitude-derived frequency for the window.
// mag_freq: scalar frequency computed from the magnitude (or 0.0 if no/weak motion).
// Returns true when persistent dyskinesia is detected (>= MIN_WINDOWS_IN_BAND in last FREQ_HISTORY_LEN windows).

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

    // // 3) If we don't have enough windows yet, we can't make a strong statement
    // if (history_count < FREQ_HISTORY_LEN) {
    //        // "not yet sure"
    // }

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

     // ---------------------------------------------------------
    // BUG FIX #1: If stddev too small → no movement, skip FFT
    // BUG FIX #2: If stddev below threshold → weak movement, ignore
    // ---------------------------------------------------------
    const float STDDEV_MIN     = 1.0f;   // absolutely still
    const float STDDEV_WEAK    = 5.0f;   // noise-level motion

    if (stddev < STDDEV_MIN) {
        return 0.0f; // no movement at all
    }

    if (stddev < STDDEV_WEAK) {
        return 0.0f; // too weak for meaningful oscillation
    }
    // ---------------------------------------------------------

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

// int16_t* magnitude(int16_t* v1, int16_t* v2, int16_t* v3) {
//     int16_t mag_buffer[BUFFER_SIZE];
//     for(int16_t i = 0  ; i < BUFFER_SIZE ; i++) {
//         //printf("x: v1[i] = %d, v2[i] = %d, v3[i] = %d\n", v1[i], v2[i], v3[i]);
//         mag_buffer[i] = sqrtf(
//             v1[i]*v1[i] +
//             v2[i]*v2[i] +
//             v3[i]*v3[i]
//         );
//     }

//     return mag_buffer;
// }

 int main() {
     // Setup I2C at 400kHz
     i2c.frequency(400000);
     
     // Check if sensor is connected
     uint8_t id = read_register(WHO_AM_I);
     // printf("WHO_AM_I = 0x%02X (Expected: 0x6A)\r\n", id);
     
     if (id != 0x6A) {
         // printf("Error: LSM6DSL sensor not found!\r\n");
         while (1) { /* Stop here */ }
     }
     
    //  // Configure the accelerometer (104 Hz, ±2g range)
    // write_register(CTRL1_XL, 0x40);
    write_register(CTRL1_XL, 0x24);  // 0010 0100 → ODR = 26 Hz, ±16g

    // printf("Accelerometer configured: 104 Hz, ±2g range\r\n");

     // 4. For ±16g range
    // write_register(CTRL1_XL, 0x44);  // 0100 0100: ODR=104Hz, FS=±16g
    write_register(CTRL1_XL, 0x24);  // 0010 0100 → ODR = 26 Hz, ±16g

    const float ACC_SENSITIVITY = 0.488f;  // mg/LSB
     
     // Configure the gyroscope (104 Hz, ±250 dps range)
     write_register(CTRL2_G, 0x40);
     int16_t acc_x_buffer[BUFFER_SIZE];
     int16_t acc_y_buffer[BUFFER_SIZE];
     int16_t acc_z_buffer[BUFFER_SIZE];
     size_t idx = 0;
     DigitalOut detect_led(LED1);   // LED shows persistent detection
     // printf("Gyroscope configured: 104 Hz, ±250 dps range\r\n");
     
     // Conversion factors for ±2g and ±250 dps
    //  const float ACC_SENSITIVITY = 0.061f;  // mg/LSB for ±2g range
     const float GYRO_SENSITIVITY = 8.75f;  // mdps/LSB for ±250 dps range
     
     // Main loop
     while (1) {

        // --- DEBUG: measure loop timing ---
        // static uint32_t last = 0;
        // uint32_t now = Kernel::get_ms_count();
        // printf("Loop dt = %lu ms\r\n", now - last);
        // last = now;
        // -----------------------------------

         // Read raw accelerometer values
         int16_t acc_x_raw = read_16bit_value(OUTX_L_XL, OUTX_H_XL);
         int16_t acc_y_raw = read_16bit_value(OUTY_L_XL, OUTY_H_XL);
         int16_t acc_z_raw = read_16bit_value(OUTZ_L_XL, OUTZ_H_XL);
         
        // --- DEBUG: Print raw Z axis ---
        // printf("acc_z_raw = %d\r\n", acc_z_raw);
        // --------------------------------

         // take z axis
         acc_x_buffer[idx] = acc_x_raw;
         acc_y_buffer[idx] = acc_y_raw;
         acc_z_buffer[idx++] = acc_z_raw;


         // printf("before if");

         // printf("%zu",idx);
         
         if(idx >= BUFFER_SIZE) {

            // --- DEBUG: measure how long it took to collect 128 samples ---
            // static uint32_t last_buffer_time = 0;
            // uint32_t now2 = Kernel::get_ms_count();
            // printf("Buffer filled in %lu ms\r\n", now2 - last_buffer_time);
            // last_buffer_time = now2;
            // --------------------------------------------------------------
            // int16_t* acc_magnitude_buffer = magnitude(
            //     acc_x_buffer,
            //     acc_y_buffer,
            //     acc_z_buffer
            // );
            // 1) Estimate dominant freq for this window
            // float freq = estimate_frequency(acc_z_buffer, BUFFER_SIZE);
            // float fx = estimate_frequency(acc_x_buffer, BUFFER_SIZE);
            // float fy = estimate_frequency(acc_y_buffer, BUFFER_SIZE);
             float fz = estimate_frequency(acc_z_buffer, BUFFER_SIZE);
            // float freq_magnitude = magnitude(fx, fy, fz);
            // float freq_magnitude = estimate_frequency(
            //     acc_magnitude_buffer,
            //     BUFFER_SIZE
            // );


            // bool in_x = (fx >= DYSK_MIN_FREQ && fx <= DYSK_MAX_FREQ);
            // bool in_y = (fy >= DYSK_MIN_FREQ && fy <= DYSK_MAX_FREQ);
            // bool in_z = (fz >= DYSK_MIN_FREQ && fz <= DYSK_MAX_FREQ);
            // bool bool_dys = (freq_magnitude >= DYSK_MIN_FREQ && freq_magnitude <= DYSK_MAX_FREQ);
            // // combined condition: all three axes in band for this window
            // bool combined_in_band = in_x || in_y || in_z;
            // bool combined_in_band = bool_dys;
            
            

            // float avg_f = 0.0f;
            // size_t count_nonzero = 0;
            // // if (fx > 0.0f) { avg_f += fx; count_nonzero++; }
            // // if (fy > 0.0f) { avg_f += fy; count_nonzero++; }
            // // if (fz > 0.0f) { avg_f += fz; count_nonzero++; }
            // if (freq_magnitude > 0.0f) { avg_f += freq_magnitude; count_nonzero++; }
            // if (count_nonzero > 0) avg_f /= count_nonzero;

            bool persistent_dys = update_and_check_persistent_dysk(fz);
            printf("\nDyskinesia check:\n");
            
            // printf("Window freqs: X=%.2fHz Y=%.2fHz Z=%.2fHz | in_band: X=%c Y=%c Z=%c | persistent dyskinesia=%s\r\n",
            //        fx, fy, fz,
            //        in_x ? 'Y' : 'N', in_y ? 'Y' : 'N', in_z ? 'Y' : 'N',
            //        persistent_dys ? "YES" : "NO");
            printf("Window mag freq: %.2fHz |  persistent dyskinesia=%s\r\n",
                   fz,

                   persistent_dys ? "YES" : "NO");



            // printf("Estimated oscillation frequency: %.2f Hz\r\n", freq);
            idx = 0; // reset buffer
            // 2) Update history & check persistence
            bool persistent_tremor = update_and_check_persistent_oscillation(fz);
            // 3) Print what’s going on
            printf("\n\nTremor check:\n");
            printf("Window freq: %.2f Hz | persistent(2.5-5.5Hz) = %s\r\n",
               fz,
               persistent_tremor ? "YES" : "NO");
            // 4) Visual indicator
            // detect_led = persistent ? 1 : 0;
            // // 5) Reset buffer index for next window
            idx = 0;


            bool fog_detected = update_and_check_fog(fz);
            printf("FOG Detection check:\n");
            printf("\nWindow freq: %.2f Hz | Dyskinesia=%s | Tremor=%s | FOG=%s\r\n",
       fz,
       persistent_dys ? "YES" : "NO",
       persistent_tremor ? "YES" : "NO",
       fog_detected ? "YES" : "NO");








        }
        
        //  // Read raw gyroscope values
        //  int16_t gyro_x_raw = read_16bit_value(OUTX_L_G, OUTX_H_G);
        //  int16_t gyro_y_raw = read_16bit_value(OUTY_L_G, OUTY_H_G);
        //  int16_t gyro_z_raw = read_16bit_value(OUTZ_L_G, OUTZ_H_G);
         
         // Convert accelerometer values from raw to g
         float acc_x_g = acc_x_raw * ACC_SENSITIVITY / 100.0f;
         float acc_y_g = acc_y_raw * ACC_SENSITIVITY / 100.0f;
         float acc_z_g = acc_z_raw * ACC_SENSITIVITY / 100.0f;
         
        //  // Convert gyroscope values from raw to dps
        //  float gyro_x_dps = gyro_x_raw * GYRO_SENSITIVITY / 1000.0f;
        //  float gyro_y_dps = gyro_y_raw * GYRO_SENSITIVITY / 1000.0f;
        //  float gyro_z_dps = gyro_z_raw * GYRO_SENSITIVITY / 1000.0f;
         
        //  // Print converted values using printf
        //  printf("Accel [g]: X=%+6.3f, Y=%+6.3f, Z=%+6.3f | Gyro [dps]: X=%+7.2f, Y=%+7.2f, Z=%+7.2f\r\n", 
        //  acc_x_g, acc_y_g, acc_z_g, gyro_x_dps, gyro_y_dps, gyro_z_dps);
         
        //  // Output Teleplot format directly with printf
        //  printf(">acc_x:%.3f\n>acc_y:%.3f\n>acc_z:%.3f\n"
        // ">gyro_x:%.2f\n>gyro_y:%.2f\n>gyro_z:%.2f\n",
        // acc_x_g, acc_y_g, acc_z_g,
        // gyro_x_dps, gyro_y_dps, gyro_z_dps);
         
         // Wait before next sample
         ThisThread::sleep_for(50ms);
     }
 }
 
