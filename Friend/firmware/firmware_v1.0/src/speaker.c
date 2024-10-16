#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/gpio.h>
#include <math.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include "speaker.h"

LOG_MODULE_REGISTER(speaker, CONFIG_LOG_DEFAULT_LEVEL);

#define MAX_BLOCK_SIZE 16000

#define BLOCK_COUNT 4
#define SAMPLE_FREQUENCY 8000
#define NUMBER_OF_CHANNELS 2
#define PACKET_SIZE 400
#define WORD_SIZE 16
#define NUM_CHANNELS 2

#define PI 3.14159265358979323846

#define MAX_HAPTIC_DURATION 5000
K_MEM_SLAB_DEFINE_STATIC(mem_slab, MAX_BLOCK_SIZE, BLOCK_COUNT, 4);

static struct device *audio_speaker;

static void* buzz_buffer;
static int16_t* clear_ptr;

static uint16_t current_length;
static uint16_t offset;

static int16_t* pointer_array[BLOCK_COUNT];
uint8_t pointer_array_index = 0;
uint16_t pointer_array_size = 0;
struct gpio_dt_spec haptic_gpio_pin = {.port = DEVICE_DT_GET(DT_NODELABEL(gpio1)), .pin=11, .dt_flags = GPIO_INT_DISABLE};

int speaker_init() 
{
    LOG_INF("Speaker init");
    audio_speaker = device_get_binding("I2S_0");
    
    if (!device_is_ready(audio_speaker)) 
    {
        LOG_ERR("Speaker device is not supported : %s", audio_speaker->name);
        return -1;
    }
    struct i2s_config config = {
    .word_size= WORD_SIZE, //how long is one left/right word.
    .channels = NUMBER_OF_CHANNELS, //how many words in a frame 2 
    .format = I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED, //format
    // .format = I2S_FMT_DATA_FORMAT_I2S,
    .options = I2S_OPT_FRAME_CLK_MASTER | I2S_OPT_BIT_CLK_MASTER | I2S_OPT_BIT_CLK_GATED, //how to configure the mclock
    .frame_clk_freq = SAMPLE_FREQUENCY, /* Sampling rate */ 
    .mem_slab = &mem_slab,/* Memory slab to store rx/tx data */
    .block_size = MAX_BLOCK_SIZE,/* size of ONE memory block in bytes */
    .timeout = -1, /* Number of milliseconds to wait in case Tx queue is full or RX queue is empty, or 0, or SYS_FOREVER_MS */
    };
    int err = i2s_configure(audio_speaker, I2S_DIR_TX, &config);
	if (err) 
    {
		LOG_ERR("Failed to configure Speaker (%d)", err);
        return -1;
	}
    for (int i = 0; i < BLOCK_COUNT; i++)
    {
        err = k_mem_slab_alloc(&mem_slab, &pointer_array[i], K_MSEC(200));
        if (err) 
        {
            printk("Failed to allocate for sound buffer (%d)\n", err);
            return -1;
        }
        memset(pointer_array[i], 0, MAX_BLOCK_SIZE);
    }

    return 0;
}

void generate_gentle_chime(int16_t *buffer, int num_samples)
{
    LOG_INF("Generating gentle chime");//2500
    const float frequencies[] = {523.25, 659.25, 783.99, 1046.50}; // C5, E5, G5, C6
    const int num_freqs = sizeof(frequencies) / sizeof(frequencies[0]);//4

    for (int i = 0; i < num_samples; i++) 
    { 
        float t = (float)i / SAMPLE_FREQUENCY;//0.000125
        float sample = 0;
        for (int j = 0; j < num_freqs; j++) 
        {
           sample += sinf(2 * PI * frequencies[j] * t) * (1.0 - t);
        }
        int16_t int_sample = (int16_t)(sample / num_freqs * 32767 * 0.5);
        buffer[i * NUM_CHANNELS] = int_sample;
        buffer[i * NUM_CHANNELS + 1] = int_sample;
    }
    LOG_INF("Done generating gentle chime");
}

int play_boot_sound(void)
{
    int ret;
    int16_t *buffer = (int16_t *) pointer_array[0];
    const int samples_per_block = MAX_BLOCK_SIZE / (NUM_CHANNELS * sizeof(int16_t));

    generate_gentle_chime(buffer, samples_per_block);
    LOG_INF("Writing to speaker");
    k_sleep(K_MSEC(100));
    ret = i2s_write(audio_speaker, buffer, MAX_BLOCK_SIZE);
    if (ret) 
    {
        LOG_ERR("Failed to write initial I2S data: %d", ret);
        return ret;
    }

    ret = i2s_trigger(audio_speaker, I2S_DIR_TX, I2S_TRIGGER_START);
    if (ret) 
    {
        LOG_ERR("Failed to start I2S transmission: %d", ret);
        return ret;
    }  


    ret = i2s_trigger(audio_speaker, I2S_DIR_TX, I2S_TRIGGER_DRAIN);
    if (ret != 0) 
    {
        LOG_ERR("Failed to drain I2S transmission: %d", ret);
        return ret;
    }

    k_sleep(K_MSEC(1000));  
    memset(pointer_array[0], 0, MAX_BLOCK_SIZE);
    return 0;
}

int init_haptic_pin() 
{
    if (gpio_is_ready_dt(&haptic_gpio_pin)) 
    {
		LOG_INF("Haptic Pin ready");
	}
    else 
    {
		LOG_ERR("Error setting up Haptic Pin");
        return -1;
	}
	if (gpio_pin_configure_dt(&haptic_gpio_pin, GPIO_OUTPUT_INACTIVE) < 0) 
    {
		LOG_ERR("Error setting up Haptic Pin");
        return -1;
	}
    gpio_pin_set_dt(&haptic_gpio_pin, 0);

    return 0;
}

void haptic_timer_callback(struct k_timer *timer);

K_TIMER_DEFINE(my_status_timer, haptic_timer_callback, NULL);

void haptic_timer_callback(struct k_timer *timer)
{
    gpio_pin_set_dt(&haptic_gpio_pin, 0);
}

void play_haptic_milli(uint32_t duration)
{
    if (duration > MAX_HAPTIC_DURATION)
    {
        LOG_ERR("Duration is too long");
        return;
    }
    gpio_pin_set_dt(&haptic_gpio_pin, 1);
    k_timer_start(&my_status_timer, K_MSEC(duration), K_NO_WAIT);
}

// uint8_t pointer_array_index = 0;
// uint16_t pointer_array_size = 0;
bool speak_started = 0;
uint16_t speak(uint16_t len, const void *buf)
{
    int res;
    if (len == 4) return len;
    
    int16_t *ptr2 = pointer_array[pointer_array_index];
    for (int i = 0; i < (int)(len/2); i++) 
    {
        ptr2[pointer_array_size] = ((int16_t *)buf)[i];  
        ptr2[pointer_array_size+1] = ((int16_t *)buf)[i]; 
        pointer_array_size = pointer_array_size + 2;
    }
    if (pointer_array_size == (int)(MAX_BLOCK_SIZE / 2))
    {
       int e =  i2s_write(audio_speaker, ptr2, MAX_BLOCK_SIZE);
       if (e)
       {
            printk("Failed to write I2S data: %d\n", e);
       }
        pointer_array_index = ( pointer_array_index + 1 ) % 4;
        pointer_array_size = 0;
    }
    if (pointer_array_index == 3 && !speak_started)
    {
        printk("Starting to speak\n");
        speak_started = true;
        k_sleep(K_MSEC(10));
        printk("Starting to speak\n");
        res = i2s_trigger(audio_speaker, I2S_DIR_TX, I2S_TRIGGER_START);
        if (res)
        {
            printk("Failed to start I2S transmission: %d\n", res);
        }
    }
    if (len != 400)
    {
        printk("Starting to speak\n");
        k_sleep(K_MSEC(10));
        res =  i2s_write(audio_speaker, ptr2, MAX_BLOCK_SIZE);
        if (res)
        {
            printk("Failed to write I2S data again: %d\n", res);
        }
        if (!speak_started)
        {
            res = i2s_trigger(audio_speaker, I2S_DIR_TX, I2S_TRIGGER_START);
            if (res)
            {
                printk("Failed to start I2S transmission: %d\n", res);
            }
        }


        res = i2s_trigger(audio_speaker, I2S_DIR_TX, I2S_TRIGGER_DRAIN);
        if (res)
        {
            printk("Failed to start I2S drain: %d\n", res);
        }

        // for (int i = 0; i < BLOCK_COUNT; i++)
        // {
        //     k_sleep(K_MSEC(500));
        //     memset(pointer_array[i], 0, MAX_BLOCK_SIZE);

        // }

        speak_started = false;    

        pointer_array_index = 0;
        pointer_array_size = 0;   
    }

    return len;

}

