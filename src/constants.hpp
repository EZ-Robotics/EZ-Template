// ports
#define LEFT_BACK 18
#define LEFT_FRONT 12
#define RIGHT_BACK 20
#define RIGHT_FRONT 14
#define IMU 17
#define INTAKE 19
#define CATA 11
#define LIMIT 'b'
#define WINGS 'a'

// autos
#define DRIVE_SPEED 110  // This is 110/127 (around 87% of max speed).  We don't suggest making this 127.
                         // If this is 127 and the robot tries to heading correct, it's only correcting by
                         // making one side slower.  When this is 87%, it's correcting by making one side
                         // faster and one side slower, giving better heading correction.
#define TURN_SPEED 90
#define SWING_SPEED 90

// main
#define CATAVOLTAGE 127
#define CATAHOLDVOLTAGE 0