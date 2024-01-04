#include "Core.h"
#include <CmdParser.hpp>

/*
 * TODO:
 * task scheduler
 */

CmdParser cmdParser;
void cmdParserRoutine();

void setup() {
  SERIAL_PORT.begin(57600);

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

  cmdParserRoutine();
};

void cmdParserRoutine() {
  if (SERIAL_PORT.available()) {
    char line[128];
    size_t lineLength = SERIAL_PORT.readBytesUntil('\n', line, 127);
    line[lineLength] = '\0';

    if (cmdParser.parseCmd(line) != CMDPARSER_ERROR) {

      if (cmdParser.equalCommand("SET_ARM") && (cmdParser.getParamCount() == 1)) { 
        if (strncmp(cmdParser.getCmdParam(1), "arm", 3) == 0) {
          log_i("Force arming");
          Core::set_armed(true); return;
        }
        else if (strncmp(cmdParser.getCmdParam(1), "disarm", 6) == 0) {
          log_i("Force disarming");
          Core::set_armed(false);
          Core::set_state(AGENT_STATE::INITED); return;
        }
      }
      #ifndef SERVER
      else if (cmdParser.equalCommand("CALI") && (cmdParser.getParamCount() == 1)) {
        // ESC calibration
        if (strncmp(cmdParser.getCmdParam(1), "1", 1) == 0) {
          if (Core::get_current_state() != AGENT_STATE::LOST_CONN) {
            log_e("Please disconnect the server first");
            return;
          }

          SERIAL_PORT.println(F(
            "\n\n=====================================\n"));
          // Raising throttle to max
          Core::esc_p1.set_armed(true);
          Core::esc_p2.set_armed(true);
          Core::esc_p1.calibrate(1);
          Core::esc_p2.calibrate(1);
          SERIAL_PORT.println(F(
            "Throttle max, please connect battery to ESC, and then enter \"CALI 2\""
            "after hearing double beeps.\n"));
          return;
        }
        else if (strncmp(cmdParser.getCmdParam(1), "2", 1) == 0) {
          // Lowering throttle to min
          Core::esc_p1.set_armed(false);
          Core::esc_p2.set_armed(false);
          Core::esc_p1.calibrate(2);
          Core::esc_p2.calibrate(2);
          SERIAL_PORT.println(F(
            "ESC configuration complete!\n"));
          return;
        }
      }
      #endif

      // print usage
      SERIAL_PORT.println(F(
        "Usage: \n\n"
        "SET_ARM [options]\n"
        "- arm\n"
        "- disarm\n\n"
      #ifndef SERVER
        "CALI [options]\n"
        "Note: Disconnect the server before calibrating ESCs!"
        "- 1: throttle max\n"
        "- 2: throttle min\n"
      #endif
      ));
    }
  }
}
