#include "CommProtocol.h"

CommProtocol::CommProtocol(){
  _ctrl_callback = NULL; 
  _state_callback = NULL;
  _instruct_callback = NULL;
}

void CommProtocol::init(){
  assert(_ctrl_callback != NULL);
  assert(_state_callback != NULL);
  assert(_instruct_callback != NULL);
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

int CommProtocol::get_packet_len(const PACKET_TYPE type) {
  uint32_t packet_len = 0;
  switch (type) {
  case PACKET_TYPE::CTRL_CMDS:
    packet_len = sizeof(CtrlPacket);
    // CtrlPacket *t_packet = dynamic_cast<CtrlPacket *>(packet);
    break;

  case PACKET_TYPE::CTRL_INST:
    packet_len = sizeof(CtrlPacket);
    break;

  case PACKET_TYPE::STATE_AGN:
    packet_len = sizeof(StatePacket);
    break;

  case PACKET_TYPE::MSG:
    packet_len = sizeof(MsgPacket);
    break;

  default:
    packet_len = sizeof(Packet);
  }
  return packet_len;
}

void CommProtocol::callback_router(uint8_t *payload, size_t length){
  // The situation of packets having the same size is not handled
  switch (length * sizeof(uint8_t)) {
    case sizeof(MsgPacket):
      break;

    case sizeof(CtrlPacket): {
      CtrlPacket ctrl_packet;
      memcpy((void *)&ctrl_packet, payload, length * sizeof(uint8_t));
      _ctrl_callback(ctrl_packet);
      break;
    }

    case sizeof(StatePacket): {
      StatePacket state_packet;
      memcpy((void *)&state_packet, payload, length * sizeof(uint8_t));
      _state_callback(state_packet);
      break;
    }

    case sizeof(InstructPacket): {
      InstructPacket instruct_packet;
      memcpy((void *)&instruct_packet, payload, length * sizeof(uint8_t));
      _instruct_callback(instruct_packet);
    }

    case sizeof(Packet):
    default:
      break;
    }
}