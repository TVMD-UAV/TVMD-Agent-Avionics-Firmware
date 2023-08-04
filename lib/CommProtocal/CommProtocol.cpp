#include "CommProtocol.h"

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
