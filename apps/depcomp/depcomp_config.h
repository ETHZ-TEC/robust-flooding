
/**
 * defines for the competition
 */

#ifndef COMPETITION_MODE
#define COMPETITION_MODE    0
#endif /* COMPETITION_MODE */

#define SINK_NODE_ID        219
#define SENSING_NODE_ID     116

#define LIGHT_SENSOR_THRES  450

#if COMPETITION_MODE
 #define INITIATOR_NODE_ID  SENSING_NODE_ID
 #define SINK_REPORT_PIN    PORT2,PIN3
 #define USE_LIGHT_SENSOR   1
#else
 #define SINK_REPORT_PIN    FLOCKLAB_INT2
#endif /* COMPETITION_MODE */


/**
 * general parameters
 */

#ifndef GLOSSY_PERIOD_MS
#define GLOSSY_PERIOD_MS    100
#endif /* GLOSSY_PERIOD_MS */

#ifndef N_TX
#define N_TX                3
#endif /* N_TX */

#ifndef N_CH
#define N_CH                3
#endif /* N_CH */

#ifndef SLEEP_BTW_EVENTS
#define SLEEP_BTW_EVENTS    0
#endif /* SLEEP_BTW_EVENTS */

#ifndef INITIATOR_NODE_ID
#define INITIATOR_NODE_ID   1
#endif /* INITIATOR_NODE_ID */

#ifndef PRINT_METRICS
#define PRINT_METRICS       0
#endif /* PRINT_METRICS */