#pragma once

#ifndef NODE_ID
#define NODE_ID 1   // default when build_flags does not provide NODE_ID
#endif

#ifndef NUM_NODES
#define NUM_NODES 4
#endif

// Bring-up mode: one node initiates ranging, all others only respond.
#ifndef INITIATOR_NODE_ID
#define INITIATOR_NODE_ID 1
#endif

// Stable debug mode: initiator ranges only to one target with low traffic.
#ifndef SINGLE_PAIR_DEBUG_MODE
#define SINGLE_PAIR_DEBUG_MODE 1
#endif

#ifndef DEBUG_TARGET_NODE_ID
#define DEBUG_TARGET_NODE_ID 2
#endif

// slot duration (ms)
#ifndef SLOT_TIME_MS
#define SLOT_TIME_MS 30
#endif

// Delay between initiator ranging attempts in bring-up mode.
#ifndef RANGING_PERIOD_MS
#define RANGING_PERIOD_MS 120
#endif

// Lower traffic and add retries for stability during bring-up.
#ifndef RANGING_PERIOD_DEBUG_MS
#define RANGING_PERIOD_DEBUG_MS 800
#endif

#ifndef MAX_INITIATOR_RETRIES
#define MAX_INITIATOR_RETRIES 3
#endif

// TDMA pairs
struct Pair {
    uint8_t a;
    uint8_t b;
};

static const Pair schedule[] = {
    {1,2},
    {1,3},
    {1,4},
    {2,3},
    {2,4},
    {3,4}
};

#define NUM_SLOTS (sizeof(schedule)/sizeof(schedule[0]))
