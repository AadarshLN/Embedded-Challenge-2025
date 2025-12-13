#include "mbed.h"
#include "arm_math.h" // CMSIS-DSP library for fast FFT

#define FFT_SIZE        256      // FFT Window Size
#define SAMPLE_RATE     104.0f   // Sampling Rate
#define MA_WINDOW       10       // Moving Average Window Size

// If PI is not defined
#ifndef PI
#define PI 3.14159265358979f
#endif

// Register Definitions
#define LSM6DSL_ADDR    (0x6A << 1) 
#define WHO_AM_I        0x0F        
#define CTRL1_XL        0x10        
#define CTRL2_G         0x11        
#define CTRL3_C         0x12        
#define OUTX_L_XL       0x28

// --- Threshold Settings ---
// 1. Basic Noise Gate (For Tremor)
// Used to filter out static noise or very slight shakes.
#define TREMOR_MIN_MAG  20.0f
#define TREMOR_MAX_MAG  100.0f

// 2. [NEW] Dyskinesia Energy Threshold
// Requires signal to be MUCH stronger to classify as Dyskinesia.
// Since arm waving creates high G-force, this should be high (try 3.0 to 5.0).
#define DYS_MIN_MAG     30.0f

I2C i2c(PB_11, PB_10);
BufferedSerial serial_port(USBTX, USBRX, 115200);
FileHandle *mbed::mbed_override_console(int) {
    return &serial_port;
}

// FFT Buffers
arm_rfft_fast_instance_f32 S;
float fft_input[FFT_SIZE];
float fft_output[FFT_SIZE];
float fft_mag[FFT_SIZE / 2];

void write_reg(uint8_t reg, uint8_t val) {
    char data[2] = {(char)reg, (char)val};
    i2c.write(LSM6DSL_ADDR, data, 2);
}

void read_all_axes(int16_t *x, int16_t *y, int16_t *z) {
    char reg = OUTX_L_XL;
    char data[6]; 
    i2c.write(LSM6DSL_ADDR, &reg, 1, true);
    i2c.read(LSM6DSL_ADDR, data, 6);
    *x = (int16_t)((data[1] << 8) | data[0]);
    *y = (int16_t)((data[3] << 8) | data[2]);
    *z = (int16_t)((data[5] << 8) | data[4]);
}

bool read_reg(uint8_t reg, uint8_t &val) {
    char r = (char)reg;
    if (i2c.write(LSM6DSL_ADDR, &r, 1, true) != 0) return false;
    if (i2c.read(LSM6DSL_ADDR, &r, 1) != 0) return false;
    val = (uint8_t)r;
    return true;
}

int16_t read_int16(uint8_t reg_low) {
    uint8_t lo, hi;
    read_reg(reg_low, lo);
    read_reg(reg_low + 1, hi);
    return (int16_t)((hi << 8) | lo);
}

bool init_sensor() {
    uint8_t who;
    if (!read_reg(WHO_AM_I, who) || who != 0x6A) {
        printf("Sensor not found! WHO_AM_I = 0x%02X\r\n", who);
        return false;
    }
    // BDU enabled
    write_reg(CTRL3_C, 0x44); 
    // 104Hz, +/- 8g (Crucial for large arm movements)
    write_reg(CTRL1_XL, 0x4C); 
    write_reg(CTRL2_G, 0x00);
    ThisThread::sleep_for(100ms);
    return true;
}

int main() {
    i2c.frequency(400000);
    arm_rfft_fast_init_f32(&S, FFT_SIZE);

    if (!init_sensor()) {
        while(1) { printf("Init Failed.\n"); ThisThread::sleep_for(1s); }
    }

    printf("System Ready. Dual Thresholds Active.\n");

    int sample_idx = 0;
    const float SENSITIVITY = 0.244f; 

    while (1) {
        // Data Acquisition
        int16_t rx, ry, rz;
        read_all_axes(&rx, &ry, &rz);
        
        float ax = rx * SENSITIVITY / 1000.0f;
        float ay = ry * SENSITIVITY / 1000.0f;
        float az = rz * SENSITIVITY / 1000.0f;

        // 3-Axis Fusion (Total Energy)
        float norm = sqrtf(ax*ax + ay*ay + az*az);
        float acc_centered = norm - 1.0f;
        
        fft_input[sample_idx] = acc_centered;
        sample_idx++;

        if (sample_idx >= FFT_SIZE) {
            
            // Hann window
            for (int i = 0; i < FFT_SIZE; i++) {
                float hann = 0.5f * (1.0f - arm_cos_f32(2.0f * PI * i / (FFT_SIZE - 1)));
                fft_input[i] *= hann;
            }

            // FFT
            arm_rfft_fast_f32(&S, fft_input, fft_output, 0);
            arm_cmplx_mag_f32(fft_output, fft_mag, FFT_SIZE / 2);

            float max_val = 0.0f;
            int max_idx = 0;
            
            // [Optimization] Search only 2Hz to 10Hz
            // Index 5 (~2.0Hz) to Index 25 (~10.0Hz)
            // This prevents detecting 50Hz noise or 12Hz harmonics as Dyskinesia.
            for (int i = 5; i < 25; i++) {
                if (fft_mag[i] > max_val) {
                    max_val = fft_mag[i];
                    max_idx = i;
                }
            }

            float freq = (float)max_idx * SAMPLE_RATE / FFT_SIZE;

            // --- CLASSIFICATION LOGIC WITH ENERGY CHECK ---
            const char* status = "Normal";

            if (max_val < 15.0f) { 
                freq = 0.0f;
            }

            if (freq > 0.0f) {
                
                // Case A: TREMOR (3-5 Hz)
                if (freq >= 2.8f && freq <= 5.2f) {

                    if (max_val > TREMOR_MIN_MAG && max_val < TREMOR_MAX_MAG) {
                        status = "TREMOR";
                    } else if (max_val >= TREMOR_MAX_MAG) {
                        status = "Large Movement";
                    } else {
                        status = "Normal"; 
                    }
                } 
                
                // Case B: DYSKINESIA (5-7 Hz)
                else if (freq > 5.2f && freq <= 7.0f) {

                    if (max_val > DYS_MIN_MAG) {
                        status = "DYSKINESIA";
                    }
                }
            }

            // Output
            printf("X:%.2f Y:%.2f Z:%.2f req_Hz:%.2f Mag:%.2f Status:%s\n", 
                   ax, ay, az, freq, max_val, status);

            sample_idx = 0; 
        }

        ThisThread::sleep_for(9ms);
    }
}