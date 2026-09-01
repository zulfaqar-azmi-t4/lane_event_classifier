# Not evaluated: an input was missing, so no classification ran this cycle.
uint8 UNDEFINED = 0
uint8 LANE_FOLLOWING = 1
uint8 LANE_CHANGING = 2
uint8 ABORTING_LANE_CHANGE = 3
uint8 INTENTIONAL_LANE_CROSSING = 4
# Value 5 is retired. It was ABORTING_INTENTIONAL_LANE_CROSSING and was never published.
# Evaluated: the ego left its lane and no classifier claimed the event.
uint8 UNKNOWN = 6
uint8 state
