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
    const CtrlPacket * const ctrl_packet_ptr = reinterpret_cast<CtrlPacket *>(payload);
    _ctrl_callback(*ctrl_packet_ptr, AGENT_STATE::INITED);
    return ctrl_packet_ptr->agent_id;
  } break;

  case sizeof(StatePacket): {
    const StatePacket * const state_packet_ptr = reinterpret_cast<StatePacket *>(payload);
    _state_callback(*state_packet_ptr);
    return state_packet_ptr->agent_id;
  } break;

  case sizeof(InstructPacket): {
    const InstructPacket * const instruct_packet_ptr = reinterpret_cast<InstructPacket *>(payload);
    _instruct_callback(*instruct_packet_ptr);
    return instruct_packet_ptr->agent_id;
  } break;

  case sizeof(CtrlPacketArray): {
    // TODO: bug exists in this callback => fixed by using reinterpret_cast<>
    // I don't know why memcpy() cause Invalid instruction error
    const CtrlPacketArray * const packet_array = reinterpret_cast<CtrlPacketArray *>(payload);

    // extract the data for this agent
    // must not be the SERVER
    if (check_agent_id_valid(_agent_id))
      _ctrl_callback(packet_array->packets[_agent_id - 1], packet_array->state);
      // log_d("Agent id: %d\n", _agent_id);
    else 
      log_e("Agent id out of range: %d\n", _agent_id);
    return _agent_id;
  } break;

  case sizeof(Packet):
  default:
    break;
  }
  return 0;
}
