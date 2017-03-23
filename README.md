Robust Flooding using Back-to-Back Synchronous Transmissions with Channel-Hopping
=================================================================================
This is the source code used for the EWSN dependability competition 2017 [1]. This solution was rated first amongst 10 submissions.

Scenario
--------
A binary event has to be communicated from a source node to a sink node over a multi-hop wireless network. This communication has to involve intermediate nodes, since source and sink are not within communication range of each other. At the same time, jammers create interference in the frequency bands used for wireless communication. The solution should be energy efficient, reliable and provide low latency. A more detailed description can be found on the competition webpage [1].

Protocol Description
--------------------
This protocol uses periodic rounds of floods to propagate information in a multi-hop network. Every flood consists of consecutive (back-to-back), synchronous transmissions. Once a packet is received, nodes use a local timeout to schedule a transmission in the next slot. In the case of a blocked channel, information is not propagated. However, in the following slot, the flood can continue at the same progress level. This is different to the approach of Glossy [2], which requires for every sent packet a preceeding packet reception in order to maintain tighly aligned, synchronous transmissions.
More information about the protocol can be found in the slides in [doc/slides](doc/slides/EWSN_dependability_competition.pdf) or in [3].

How to use it
-------------
Following key parameters can be set in the Makefile for the [depcomp app](app/depcomp/):

**GLOSSY_PERIOD_MS**
Interval of floods in milliseconds. Tested values: 25, 50, 100, 200

**N_TX**
Number of transmissions per node in a flood.

**N_CH**
Number of different channels to use.

**SLEEP_BTW_EVENTS**
Set to 1 to enable the round skipping feature. If enabled, rounds following an event are skipped in order to save energy. This is only safe if the minimal interval between events is known (e.g., 2 seconds in the competition).

To run this code on FlockLab [4], set the DEFINES in the Makefile as follows:
```
DEFINES=COMPETITION_MODE=0,TINYOS_NODE_ID=1,GLOSSY_PERIOD_MS=50,N_TX=6,N_CH=6,SLEEP_BTW_EVENTS=1,USE_LIGHT_SENSOR=0
```
This allows to generate events on the initiator node using the GPIO actuation feature of FlockLab (i.e. by toggling the SIG1 pin).

Contact
-------
Roman Lim <lim@tik.ee.ethz.ch>
Reto Da Forno <rdaforno@ee.ethz.ch>
Felix Sutton <rdaforno@ee.ethz.ch>
Lothar Thiele <thiele@ethz.ch>

References
----------
* [1] EWSN 2017 Dependability Competition http://www.ewsn2017.org/dependability-competition.html
* [2] F. Ferrari, M. Zimmerling, L. Thiele, and O. Saukh. Efficient network flooding and time synchronization with Glossy. In Proceedings of the 10th ACM/IEEE International Conference on Information Processing in Sensor Networks (IPSN), 2011
* [3] R. Lim, R. Da Forno, F. Sutton, and L. Thiele. Robust Flooding using Back-to-Back Synchronous Transmissions with Channel-Hopping, Proceedings of the 14th International Conference on Embedded Wireless Systems and Networks (EWSN 2017), Uppsala, Sweden, 2017
* [4] http://flocklab.ethz.ch/
