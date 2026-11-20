/*
 * route.h - pure relay decision, the only relay logic on any node (catalog v0.8)
 *
 * Every node sends every frame on each bus where its own link is up, so
 * routing is only about relaying. A frame received on bus X is forwarded
 * to bus Y when:
 *   - it is not already RELAYED (a relayed frame is never relayed again),
 *   - this node is up on both buses (its own detection, heard[self - 1]),
 *   - the sender cannot have put it on Y itself: the committed graph or the
 *     sender's latest heartbeat (byte 2, its own bus-health bits) says it has
 *     no Y link, and
 *   - some other node needs it: it hears on Y and has no X link.
 * heard[n - 1] = bus-health bits from node n's latest heartbeat (0 if its
 * heartbeat timed out); heard[self - 1] = this node's own detection.
 */
#ifndef ROUTE_H
#define ROUTE_H

#include <stdbool.h>
#include <stdint.h>
#include "raft_core.h"

uint8_t route_tx_mask(const raft_net_t *net, uint8_t self);
uint8_t route_relay_mask(const raft_net_t *net, const uint8_t heard[3], uint8_t self,
                         uint8_t sender, uint8_t from_bus, bool already_relayed);
bool    route_reachable(const raft_net_t *net, uint8_t a, uint8_t b, uint8_t *via);

#endif /* ROUTE_H */
