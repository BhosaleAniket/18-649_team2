#include "route.h"

uint8_t route_tx_mask(const raft_net_t *net, uint8_t self)
{
    uint8_t m = (self >= 1 && self <= 3) ? (uint8_t)(net->link[self - 1] & 0x03) : 0;
    return m ? m : 0x03;                        /* unknown: try both */
}

uint8_t route_relay_mask(const raft_net_t *net, const uint8_t heard[3], uint8_t self,
                         uint8_t sender, uint8_t from_bus, bool already_relayed)
{
    if (already_relayed || self < 1 || self > 3 || sender < 1 || sender > 3 || sender == self ||
        from_bus < 1 || from_bus > 2) {
        return 0;
    }
    uint8_t x = RAFT_BUSBIT(from_bus);
    uint8_t y = RAFT_BUSBIT(3 - from_bus);

    if ((heard[self - 1] & 0x03) != 0x03) {
        return 0;                               /* this node is not on both buses */
    }
    bool sender_lacks_y = !(net->link[sender - 1] & y) || !(heard[sender - 1] & y);
    if (!sender_lacks_y) {
        return 0;
    }
    for (uint8_t n = 1; n <= 3; n++) {
        if (n == self || n == sender) {
            continue;
        }
        bool hears_y = (heard[n - 1] & y) != 0;
        bool lacks_x = !(net->link[n - 1] & x) || !(heard[n - 1] & x);
        if (hears_y && lacks_x) {
            return y;
        }
    }
    return 0;
}

bool route_reachable(const raft_net_t *net, uint8_t a, uint8_t b, uint8_t *via)
{
    if (a < 1 || a > 3 || b < 1 || b > 3) {
        return false;
    }
    uint8_t na = RAFT_NODEBIT(a), nb = RAFT_NODEBIT(b);
    if (!(net->alive & na) || !(net->alive & nb)) {
        return false;
    }
    if (net->link[a - 1] & net->link[b - 1]) {
        if (via) {
            *via = 0;
        }
        return true;
    }
    for (uint8_t c = 1; c <= 3; c++) {
        if (c == a || c == b || !(net->alive & RAFT_NODEBIT(c))) {
            continue;
        }
        if ((net->link[c - 1] & net->link[a - 1]) && (net->link[c - 1] & net->link[b - 1])) {
            if (via) {
                *via = c;
            }
            return true;
        }
    }
    return false;
}
