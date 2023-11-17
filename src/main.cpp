#include "Core.h"

/*
 * TODO:
 * task scheduler
 */


void setup() {
  Serial.begin(115200);

  Core::init();
  log_i("Program start!");

#ifdef SERVER
  InstructPacket idle_packet;
  idle_packet.instruction = InstructPacket::INSTRUCT_TYPE::IDLE;
#endif
}

void loop() {
  static time_t last_publish_time = 0;
  static time_t last_summary_time = 0;

  if (millis() - last_summary_time >= 1000) {
    Core::print_summary();
    last_summary_time = millis();
  }

#ifdef SERVER
  if (millis() - last_publish_time >= 1000) {
    Core::print_instructions();
    last_publish_time = millis();
  }
#endif
};

#endif