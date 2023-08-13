#include "CommProtocol.h"

CommProtocol::CommProtocol() {
  _ctrl_callback = NULL;
  _state_callback = NULL;
  _instruct_callback = NULL;
  _disconnect_callback = NULL;
}

void CommProtocol::init(uint8_t agent_id) {
  assert(_ctrl_callback != NULL);
  assert(_state_callback != NULL);
  assert(_instruct_callback != NULL);
  assert(_disconnect_callback != NULL);
  _agent_id = agent_id;
}

void CommProtocol::allocate_packet_by_length(Packet *packet, size_t length) {
  switch (length * sizeof(uint8_t)) {
  case sizeof(MsgPacket):
    packet = new MsgPacket();
    break;

  case sizeof(CtrlPacket):
    packet = new CtrlPacket();
    break;

  case sizeof(StatePacket):
    packet = new StatePacket();
    break;

  case sizeof(Packet):
  default:
    packet = new Packet();
    break;
  }
}

uint8_t CommProtocol::callback_router(uint8_t *payload, size_t length) {
  // The situation of packets having the same size is not handled
  switch (length * sizeof(uint8_t)) {
  case sizeof(MsgPacket):
    break;

  case sizeof(CtrlPacket): {
    CtrlPacket ctrl_packet;
    memcpy((void *)&ctrl_packet, payload, sizeof(CtrlPacket));
    _ctrl_callback(ctrl_packet);
    return ctrl_packet.id;
  } break;

  case sizeof(StatePacket): {
    StatePacket state_packet;
    memcpy((void *)&state_packet, payload, sizeof(StatePacket));
    _state_callback(state_packet);
    return state_packet.id;
  } break;

  case sizeof(InstructPacket): {
    InstructPacket instruct_packet;
    memcpy((void *)&instruct_packet, payload, sizeof(InstructPacket));
    _instruct_callback(instruct_packet);
    return instruct_packet.id;
  } break;

  case sizeof(CtrlPacketArray): {
    // TODO: bug exists in this callback
    CtrlPacketArray packet_array;
    memcpy((void *)&packet_array, payload, sizeof(CtrlPacketArray));

    // extract the data for this agent
    _ctrl_callback(packet_array.packets[_agent_id - 1]);
    return _agent_id;
  } break;

  case sizeof(Packet):
  default:
    break;
  }
  return 0;
}
