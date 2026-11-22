#include <stdio.h>
#include <stdint.h> //This allows us to get exact size integers like uint9_t (1 byte) and uint16_t (2 bytes)
#include <stdbool.h> // Give us bool, true and false
#include <pthread.h>
#include <unistd.h>
#include <time.h>


#define BUFFER_SIZE 8 // For fixed memory. it can hold 8 Can frames at a time



typedef struct{
    uint32_t id; //Sensor ID allows you to see which sensor is talking
    uint8_t length; //How many bytes are data we are using and the maximum is 8
    uint8_t data[8];  //The raw 8 bytes of payload data
} CANFrame;

typedef struct{
    CANFrame buffer[BUFFER_SIZE];
    int head;      // Write index (where new frames go IN)
    int tail;      // Read index ( old frames come out)
    int count;     //How many frames are currently waiting
    pthread_mutex_t lock; //protects head, tail,count,and buffer[] across threads
} RingBuffer;
void pack_battery_data(CANFrame *frame_ptr, uint8_t temp_celsius, uint8_t soc_percent);
void pack_motor_data(CANFrame *frame_ptr, uint16_t rpm, uint8_t throttle_percent);
uint16_t unpacked_rpm(CANFrame *frame_ptr);
void check_battery_fault(CANFrame *frame_ptr);
void check_motor_fault(CANFrame *frame_ptr);
// Function used to initialize the Ring Buffer setting everything to zero
void rb_init(RingBuffer * rb)
{
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
    pthread_mutex_init(&rb->lock, NULL);
}
bool rb_push_unlocked(RingBuffer * rb, CANFrame frame)
{
    if(rb->count == BUFFER_SIZE)
    {
        return false;
    }
    rb->buffer[rb->head] = frame;
    rb->head = (rb->head + 1) % BUFFER_SIZE;
    rb->count++;
    return true;
}
bool rb_pop_unlocked(RingBuffer * rb, CANFrame * out_frame)
{
    if(rb->count == 0)
    {
        return false;
    }
    *out_frame = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % BUFFER_SIZE;
    rb->count--;
    return true;
}
bool safe_rb_push(RingBuffer * rb, CANFrame frame)
{
    pthread_mutex_lock(&rb->lock);
    bool ok = rb_push_unlocked(rb, frame);
    pthread_mutex_unlock(&rb->lock);
    return ok;
}
bool safe_rb_pop(RingBuffer * rb, CANFrame *out_frame)
{
    pthread_mutex_lock(&rb->lock);
    bool ok = rb_pop_unlocked(rb, out_frame);
    pthread_mutex_unlock(&rb->lock);
    return ok;
}
//Thread 1: Simulates car sensors sending frames every 150ms
void * sensor_producer_thread(void * arg)
{
    RingBuffer * queue = (RingBuffer *)arg;
    for(int i = 0; i < 10; i++)
    {
        CANFrame msg;
        if(i % 2 == 0)
        {
            uint8_t temp = (i == 6) ? 65 : 42; //Spike temp on interation 6
            pack_battery_data(&msg, temp, 90);
        } else {
            uint16_t rpm = (i == 7) ? 5400 : 2800; //Spike ROM + throttle on interation 7
            uint8_t thr = (i == 7) ? 85 : 30;
            pack_motor_data(&msg, rpm, thr);
        }
        if (!safe_rb_push(queue,msg))
        {
            printf("[Producer] Queue full! Dropped ID 0x%X\n", msg.id);
        }
        else{
            printf("[Producer] Pushed ID 0x%X (iter %d)\n", msg.id, i);
        }
        usleep(150000); //150 ms
    }
    return NULL;
}
// Thread 2: Acts as the safety ECU draining and checking frames
void * ecu_consumer_thread(void *arg)
{
    RingBuffer * queue = (RingBuffer *)arg;
    CANFrame rx_frame;

    time_t now;
    time_t last_battery_time = 0;
    time_t last_motor_time = 0;
    bool seen_battery = false;
    bool seen_motor = false;
    bool battery_timed_out = false;
    bool motor_timed_out = false;



    for (int checks = 0; checks < 100; checks++){
        if(safe_rb_pop(queue, &rx_frame))
        {
            now = time(NULL);
            printf("[ECU] Popped ID 0x%X -> ", rx_frame.id);
            if (rx_frame.id == 0x101)
            {
                last_battery_time = now;
                seen_battery = true;
                battery_timed_out = false;
                printf("Batt Temp: %u C | ", rx_frame.data[0]);
                check_battery_fault(&rx_frame);
            } else if (rx_frame.id == 0x102) {
                last_motor_time = now;
                seen_motor = true;
                motor_timed_out = false;
                printf("RPM: %u, Thr: %u%% | ", unpacked_rpm(&rx_frame), rx_frame.data[2]);
                check_motor_fault(&rx_frame);
            }

        }
        now = time(NULL);
        //After the producer thread finishes, check if > 2 pass with no updates
        if (seen_battery && !battery_timed_out && difftime(now, last_battery_time) > 2.0)
        {
            printf("[WATCHDOG] FAULT: No Battery (0x101) CAN frame for >2s! Check harness/BMS.\n");
            battery_timed_out = true;
        }
        if (seen_motor && !motor_timed_out && difftime(now, last_motor_time) > 2.0)
        {
            printf("[WATCHDOG] FAULT: No Motor (0x102) CAN frame for >2s! Check inverter/CAN bus.\n");
            motor_timed_out = true;
        }
        usleep(60000);
    }
    return NULL;
}
/*
bool rb_push(RingBuffer * rb, CANFrame frame)
{
    if(rb->count == BUFFER_SIZE)
    {
        return false;  //This means that the buffer is full, we shouldnt overwrite unread safety data!
    }
    //Write the frame at the head slot
    rb->buffer[rb->head] = frame;
    // Move head foward by 1, wrapping around to 0 if it hits BUFFER_SIZE (8)
    rb->head = (rb->head + 1) % BUFFER_SIZE;
    rb->count++;
    return true;
}
bool rb_pop(RingBuffer * rb, CANFrame * out_frame)
{
    if(rb->count == 0)
    {
        return false;
    }
    // *out_frame means put this frame inside the box that out_frame points to"
    *out_frame = rb->buffer[rb->tail];
    // Move tail foward by 1, wrapping around to 0 if hits BUFFER_SIZE(8)
    rb->tail = (rb->tail + 1) % BUFFER_SIZE;
    rb->count--;
    return true;
}
*/
// This function has a pointer that points to the CANFrame struct
// *frame_ptr is holding the ram address
// -> Go to memory address
void pack_battery_data(CANFrame * frame_ptr, uint8_t temp_celsius, uint8_t soc_percent)
{
    frame_ptr->id = 0x101; //0x101 = Hexadecimal iD for our battery sensor
    frame_ptr->length = 2; //Sending 2 bytes of data
    frame_ptr->data[0] = temp_celsius; //Byte 0 Battery temperature
    frame_ptr->data[1] = soc_percent;  // Which is the state of the battery
}

//PACK: Slice a 16 bit RPM (0 to 65,535) and a 16 bit wheel speed into raw CAN Bus
void pack_motor_data(CANFrame * frame_ptr, uint16_t rpm, uint8_t throttle_percent)
{
    frame_ptr->id = 0x102;  // 0x102 = Motor Controller id
    frame_ptr->length = 3;  // 2 bytes for RPM + 1 byte for throttle

    frame_ptr->data[0] = (rpm >> 8) & 0xFF; //High Byte (top 8 bits of RPM)
    frame_ptr->data[1] = rpm & 0xFF;  //Low byte(bottom 8 bits of RPM)
    frame_ptr->data[2] = throttle_percent;  //Byte 2: Throotle pedal (0-100%)

}
//Gluing back together 
uint16_t unpacked_rpm(CANFrame * frame_ptr)
{
    uint16_t high_byte = frame_ptr->data[0];
    uint16_t low_byte = frame_ptr->data[1];

    uint16_t combined_rpm = (high_byte << 8) | low_byte;
    return combined_rpm;
}
void check_battery_fault(CANFrame * frame_ptr)
{
    if(frame_ptr->data[0] > 60)
    {
        printf("CRITICAL FAULT: BATTERY OVERHEATING! SHUTTING DOWN LOW VOLTAGE CIRCUIT!\n");
    }
}
void check_motor_fault(CANFrame * frame_ptr)
{
    uint16_t rpm;
    rpm = unpacked_rpm(frame_ptr);
    if(rpm > 5000 && frame_ptr->data[2] > 75)
    {
        printf("WARNING: High RPM + High Throttle Plausibility Fault!\n");
    }
}
int main()
{
    /*CANFrame battery_msg;
    CANFrame motor_msg;
    CANFrame rx_frame;
    uint16_t unpacked;
    */
    RingBuffer can_queue;
    rb_init(&can_queue);
    pthread_t producer_tid, consumer_tid;

    //Start both threads passing &can_queue as their shared argument

    pthread_create(&producer_tid, NULL, sensor_producer_thread, &can_queue);
    pthread_create(&consumer_tid, NULL, ecu_consumer_thread, &can_queue);

    //Waiting for both threads to finish before exiting main()
    pthread_join(producer_tid, NULL);
    pthread_join(consumer_tid, NULL);

    pthread_mutex_destroy(&can_queue.lock);
    printf("Done.\n");



    /*printf("Memory Address of battery_msg: %p\n", (void*)&battery_msg);
    printf("Size of CANFrame in bytes:     %lu bytes\n\n", sizeof(CANFrame));

    pack_battery_data(&battery_msg,65,92); //This is passing in the address of the battery_msg struct so the function modifes that box we are working with
    check_battery_fault(&battery_msg);
    pack_motor_data(&motor_msg,5200,80);  // 5200 RPM, 80% Throttle
    unpacked = unpacked_rpm(&motor_msg);
    printf("Unpacked Motor RPM: %u RPM\n", unpacked);
    check_motor_fault(&motor_msg);

    rb_push(&can_queue, battery_msg);
    rb_push(&can_queue, motor_msg);

    while(rb_pop(&can_queue, &rx_frame))
    {
        printf("Popped Frame ID: 0x%X (Remaining in queue: %d)\n", rx_frame.id, can_queue.count);
        if(rx_frame.id == 0x101)
        {
            check_battery_fault(&rx_frame);
        }
        else if (rx_frame.id == 0x102)
        {
            check_motor_fault(&rx_frame);
        }
    }



    //Read the data back out from the original our original box
    printf("--- CAN Frame Recieved ---\n");
    printf("Sensor ID:      0x%X\n", battery_msg.id);
    printf("Battery Temp:   %u C\n", battery_msg.data[0]);
    printf("Battery Charge: %u %%\n", battery_msg.data[1]);
   */

    return 0;

}
