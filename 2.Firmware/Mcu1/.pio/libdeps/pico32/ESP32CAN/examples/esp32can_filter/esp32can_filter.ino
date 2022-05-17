#include <ESP32CAN.h>
#include <CAN_config.h>

CAN_device_t CAN_cfg;               // CAN Config
const int rx_queue_size = 10;       // Receive Queue size

void setup() {
  Serial.begin(115200);
  Serial.println("Filter Demo - ESP32-Arduino-CAN");
  CAN_cfg.speed = CAN_SPEED_125KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_5;
  CAN_cfg.rx_pin_id = GPIO_NUM_4;
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));

  // Set CAN Filter
  // See in the SJA1000 Datasheet chapter "6.4.15 Acceptance filter"
  // and the APPLICATION NOTE AN97076 chapter "4.1.2 Acceptance Filter" 
  // for PeliCAN Mode
  CAN_filter_t p_filter;
  p_filter.FM = Single_Mode;

  p_filter.ACR0 = 0x29;
  p_filter.ACR1 = 0;
  p_filter.ACR2 = 0;
  p_filter.ACR3 = 0;

  p_filter.AMR0 = 0;
  p_filter.AMR1 = 0xFF;
  p_filter.AMR2 = 0xFF;
  p_filter.AMR3 = 0xFF;
  ESP32Can.CANConfigFilter(&p_filter);

  // Init CAN Module
  ESP32Can.CANInit();
}

void loop() {

  CAN_frame_t rx_frame;

  unsigned long currentMillis = millis();

  // Receive next CAN frame from queue
  if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE) {

    if (rx_frame.FIR.B.FF == CAN_frame_std) {
      printf("New standard frame");
    }
    else {
      printf("New extended frame");
    }

    if (rx_frame.FIR.B.RTR == CAN_RTR) {
      printf(" RTR from 0x%08X, DLC %d\r\n", rx_frame.MsgID,  rx_frame.FIR.B.DLC);
    }
    else {
      printf(" from 0x%08X, DLC %d, Data ", rx_frame.MsgID,  rx_frame.FIR.B.DLC);
      for (int i = 0; i < rx_frame.FIR.B.DLC; i++) {
        printf("0x%02X ", rx_frame.data.u8[i]);
      }
      printf("\n");
    }
  }
}

