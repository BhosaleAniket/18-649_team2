#include "raft_proto.h"

#include <stdio.h>

const char *raft_node_name(uint8_t n)
{
    switch (n) {
    case RAFT_NODE_COCKPIT:  return "cockpit";
    case RAFT_NODE_STEERING: return "steering";
    case RAFT_NODE_DRIVE:    return "drivetrain";
    default:                 return "?";
    }
}

int raft_entry_str(uint8_t e, char *buf, int len)
{
    uint8_t n = RAFT_ENT_NODE(e);
    switch (RAFT_ENT_TYPE(e)) {
    case RAFT_ET_NOP:
        return snprintf(buf, len, "MEMBER NOP");
    case RAFT_ET_COCKPIT:
        return snprintf(buf, len, "MEMBER COCKPIT_%s", RAFT_ENT_UP(e) ? "OK" : "ERROR");
    case RAFT_ET_NODE:
        return snprintf(buf, len, "MEMBER NODE %s %s", raft_node_name(n), RAFT_ENT_UP(e) ? "UP" : "DOWN");
    case RAFT_ET_LINK:
        return snprintf(buf, len, "MEMBER LINK %s %c %s", raft_node_name(n),
                        RAFT_ENT_BUS(e) == RAFT_BUS_A ? 'A' : 'B', RAFT_ENT_UP(e) ? "UP" : "DOWN");
    case RAFT_ET_EVENT:
        return snprintf(buf, len, "EVENT %s %u%s", raft_node_name(n), RAFT_ENT_CODE(e),
                        RAFT_ENT_CODE(e) == RAFT_EVT_TEST_BUTTON ? " (test button)" : "");
    default:
        return snprintf(buf, len, "? 0x%02x", e);
    }
}
