/*
 *
 */

struct bg_movement {
  // Speed is steps/second of the axis with the highest number of steps. All
  // other axis are scaled accordingly.
  float start_speed;
  float travel_speed;
  float end_speed;

  int steps[8];   // number of steps for axis. Negative for 'backwards'
};

int beagleg_init(void); 
int beagleg_enqueue(const struct bg_movement *param);

void beagleg_wait_queue_empty(void);
void beagleg_exit(void);
