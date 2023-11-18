#ifndef PACKETS_H
#define PACKETS_H

#include <stdint.h>
#include <stdio.h>
#include "configs.h"

union Quaternion{
  struct {
    float q0;
    float q1;
    float q2;
    float q3;
  };
  float data[4];
};

union Vector3f{
  struct {
    float x;
    float y;
    float z;
  };
  float data[3];
};

struct SensorData{
  Quaternion orientation;
  Vector3f acc;
  Vector3f gyro;
  Vector3f compass;

  float temperature;
  float pressure;
  float altitude;
};

struct Packet {
  uint32_t time;
  // TODO: unique id
  uint8_t id;
  uint8_t agent_id;
  // To make Packet polymorphic, and thus a dynamic_cast can be used.
  virtual ~Packet() = default;
};

enum AGENT_STATE { STARTING = 0, INITED, LOST_CONN, ARMING, ARMED };

/* Steps to add a new packet
 * 1. Add the packet struct here
 * 2. Add new route in callback_router()
 */
struct CtrlPacket : public Packet {
  double eta_x;
  double eta_y;
  double omega_p1;
  double omega_p2;
};

struct CtrlPacketArray : public Packet {
  CtrlPacket packets[MAX_NUM_AGENTS];
};

struct StatePacket : public Packet {
  AGENT_STATE state;
  SensorData s;
};

struct InstructPacket : public Packet {
  enum INSTRUCT_TYPE { IDLE = 0, ARMING, DISARMING, EMERGENCY_STOP };
  INSTRUCT_TYPE instruction;
};

struct MsgPacket : public Packet {
  char msg[400];
};

#endif 
