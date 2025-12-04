#include "mbed.h"
#include "arm_math.h" // CMSIS-DSP library for fast FFT

#define FFT_SIZE        256      // FFT Window Size (Must be power of 2)
#define SAMPLE_RATE     104.0f   // Sampling Rate in Hz (Requirement is >52Hz)
#define MA_WINDOW       10       // Moving Average Window Size

#define LSM6DSL_ADDR    (0x6A << 1) // I2C Address shifted for Mbed
#define WHO_AM_I        0x0F        // Device Identification Register
#define CTRL1_XL        0x10        // Accelerometer Control Register
#define CTRL2_G         0x11        // Gyroscope Control Register
#define CTRL3_C         0x12        // Control Register 3 (Used for BDU)
#define OUTX_L_XL       0x28

// Initialize I2C on pins PB_11 (SDA) and PB_10 (SCL)
I2C i2c(PB_11, PB_10);

// Create serial and bind it to printf (Required for Teleplot)
BufferedSerial serial_port(USBTX, USBRX, 115200);
FileHandle *mbed::mbed_override_console(int) {
    return &serial_port;
}

// FFT Buffers
arm_rfft_fast_instance_f32 S;
float fft_input[FFT_SIZE];
float fft_output[FFT_SIZE];
float fft_mag[FFT_SIZE / 2];

// Moving Average Buffers
float ma_buffer[MA_WINDOW] = {0};
int ma_idx = 0;
float ma_sum = 0.0f;


// Write a byte to a specific register
void write_reg(uint8_t reg, uint8_t val) {
    char data[2] = {(char)reg, (char)val};
    i2c.write(LSM6DSL_ADDR, data, 2);
}

void read_all_axes(int16_t *x, int16_t *y, int16_t *z) {
    char reg = OUTX_L_XL;
    char data[6]; // X_L, X_H, Y_L, Y_H, Z_L, Z_H
    
    i2c.write(LSM6DSL_ADDR, &reg, 1, true);
    i2c.read(LSM6DSL_ADDR, data, 6);
    
    *x = (int16_t)((data[1] << 8) | data[0]);
    *y = (int16_t)((data[3] << 8) | data[2]);
    *z = (int16_t)((data[5] << 8) | data[4]);
}

// Read a single byte (Returns success/fail status)
bool read_reg(uint8_t reg, uint8_t &val) {
    char r = (char)reg;
    // Write register address
    if (i2c.write(LSM6DSL_ADDR, &r, 1, true) != 0) return false;
    // Read register value
    if (i2c.read(LSM6DSL_ADDR, &r, 1) != 0) return false;
    val = (uint8_t)r;
    return true;
}

// Read 16-bit integer (Low byte first, then High byte)
int16_t read_int16(uint8_t reg_low) {
    uint8_t lo, hi;
    read_reg(reg_low, lo);
    read_reg(reg_low + 1, hi);
    return (int16_t)((hi << 8) | lo);
}

// Initialize the LSM6DSL sensor
bool init_sensor() {
    uint8_t who;
    
    // Read WHO_AM_I register and verify it's 0x6A
    if (!read_reg(WHO_AM_I, who) || who != 0x6A) {
        printf("Sensor not found! WHO_AM_I = 0x%02X\r\n", who);
        return false;
    }

    // Configure Block Data Update (BDU)
    write_reg(CTRL3_C, 0x44); 

    // Configure Accelerometer: 104 Hz ODR, +/- 8g range, 400Hz Analog Filter
    // 0x4C = 0100 1100 (ODR=104Hz, FS=8g, BW=400Hz)
    write_reg(CTRL1_XL, 0x4C); 
    
    // Configure Gyroscope (Optional, not used in this specific demo)
    write_reg(CTRL2_G, 0x00);

    // Wait for sensor to stabilize
    ThisThread::sleep_for(100ms);
    return true;
}

int main() {
    // Setup I2C at 400kHz (Fast Mode)
    i2c.frequency(400000);

    // Initialize CMSIS-DSP FFT Library
    // Generates lookup tables for fast calculation
    arm_rfft_fast_init_f32(&S, FFT_SIZE);

    // Initialize Sensor with robust checks
    if (!init_sensor()) {
        while(1) { 
            printf("Init Failed.\n"); 
            ThisThread::sleep_for(1s); 
        }
    }

    printf("System Ready. Sample Rate: %.1f Hz\n", SAMPLE_RATE);

    int sample_idx = 0;
    
    // Sensitivity for +/- 8g range is 0.244 mg/LSB
    const float SENSITIVITY = 0.244f; 

    while (1) {
        // Data Acquisition ---
        int16_t rx, ry, rz;
        read_all_axes(&rx, &ry, &rz);
        
        // Convert to Gravity (g)
        float ax = rx * SENSITIVITY / 1000.0f;
        float ay = ry * SENSITIVITY / 1000.0f;
        float az = rz * SENSITIVITY / 1000.0f;

        // // This smooths the data for the Teleplot graph but is NOT used for FFT
        // ma_sum -= ma_buffer[ma_idx];
        // ma_buffer[ma_idx] = acc_z_g;
        // ma_sum += ma_buffer[ma_idx];
        // ma_idx = (ma_idx + 1) % MA_WINDOW;
        // float filtered_acc_z = ma_sum / MA_WINDOW;

        float norm = sqrtf(ax*ax + ay*ay + az*az);

  
        // FFT works best on signals centered at 0.
        // Z-axis typically measures 1.0g when static, so we subtract 1.0f.
        float acc_centered = norm - 1.0f;
        
        // Fill FFT Input Buffer
        fft_input[sample_idx] = acc_centered;
        sample_idx++;

        if (sample_idx >= FFT_SIZE) {
            
            // Hann window
            for (int i = 0; i < FFT_SIZE; i++) {
                float hann = 0.5f * (1.0f - arm_cos_f32(2.0f * PI * i / (FFT_SIZE - 1)));
                fft_input[i] *= hann;
            }


            // xecute FFT (Real -> Complex)
            arm_rfft_fast_f32(&S, fft_input, fft_output, 0);
            
            // Compute Magnitude (Complex -> Real)
            arm_cmplx_mag_f32(fft_output, fft_mag, FFT_SIZE / 2);

            // Find Dominant Frequency (High-Pass Filtered)
            float max_val = 0.0f;
            int max_idx = 0;
            
            // Start search from index 7 (approx 2.8Hz)
            // This ignores low-frequency movements like walking or waving arms (typically < 2Hz)
            for (int i = 1; i < FFT_SIZE / 2; i++) {
                if (fft_mag[i] > max_val) {
                    max_val = fft_mag[i];
                    max_idx = i;
                }
            }

            // Convert bin index to Frequency (Hz)
            float freq = (float)max_idx * SAMPLE_RATE / FFT_SIZE;


            // Since we converted to 'g', the magnitude values are smaller than raw data.
            // Adjust this threshold if you see non-zero frequency when static.
            #define MAG_THRESHOLD 1.2f 

            if (max_val < MAG_THRESHOLD) {
                freq = 0.0f; // Force frequency to 0 if signal is too weak
            }

            // Classification Logic
            const char* status = "Normal";
            if (freq > 0.0f) {
                // Tremor: 3 - 5 Hz 
                if (freq >= 2.8f && freq <= 5.2f) {
                    status = "TREMOR";
                } 
                // Dyskinesia: 5 - 7 Hz 
                // Expanded range slightly to 8.0Hz to capture fast movements easier
                else if (freq > 5.2f && freq <= 8.2f) {
                    status = "DYSKINESIA";
                }
            }

            // Output
            // printf("Freq_Hz:%.2f  Mag:%.2f  Status:%s\n", freq, max_val, status);
            printf(">X:%.2f >Y:%.2f >Z:%.2f >Freq_Hz:%.2f >Mag:%.2f >Status:%s\n", 
                   ax, ay, az, freq, max_val, status);

            // Reset buffer index
            sample_idx = 0; 
        }

        // Sampling Delay
        // 104 Hz = approx 9.6ms period
        ThisThread::sleep_for(9ms);
    }
}
