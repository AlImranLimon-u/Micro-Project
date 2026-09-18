#include <Servo.h>
#include <FlexiTimer2.h>

#define USE_SOFTWARE_SERIAL 0

#if USE_SOFTWARE_SERIAL
  #include <SoftwareSerial.h>
  SoftwareSerial softBT(A2, A3);
  #define LINK softBT
#else
  #define LINK Serial
#endif

#define DEBUG_TRACE 1

#if DEBUG_TRACE
  #define TRACE(x)   LINK.print(F("#")); LINK.println(F(x))
#else
  #define TRACE(x)
#endif

const unsigned long LINK_BAUD    = 9600;
const unsigned long CMD_TIMEOUT  = 900;
const unsigned long PING_PERIOD  = 120;
const unsigned long TX_PERIOD    = 350;

const int TRIG_PIN     = A2;
const int ECHO_PIN     = A3;
const int OBSTACLE_CM  = 20;

Servo servo[4][3];
const int servo_pin[4][3] = { {3, 4, 2}, {6, 7, 5}, {9, 8, 10}, {12, 11, 13} };

const float length_a    = 55;
const float length_b    = 77.5;
const float length_c    = 27.5;
const float length_side = 71;
const float z_absolute  = -28;

const float z_default = -55, z_up = -40, z_boot = z_absolute;
const float x_default = 66, x_offset = 0;
const float y_start = -0, y_step = 30;

const float y_neutral = y_start + y_step;
const float y_amp     = y_step;
const float x_amp     = 13;

#define STRAFE_INVERT 0

const int ySign[4] = { +1, -1, +1, -1 };
#if STRAFE_INVERT
  const int xSign[4] = { -1, -1, +1, +1 };
#else
  const int xSign[4] = { +1, +1, -1, -1 };
#endif

volatile float site_now[4][3];
volatile float site_expect[4][3];
float temp_speed[4][3];
float move_speed;
float speed_multiple = 1;
const float leg_move_speed   = 5.5;
const float body_move_speed  = 1.5;
const float stand_seat_speed = 1;

const float KEEP = 255;
const float pi   = 3.1415926;

float cmd_bx = 0, cmd_by = 0;
float cur_bx = 0, cur_by = 1;
bool  phaseA = true;
unsigned long last_cmd_ms = 0;
long  distance_cm = -1;

void servo_service(void);
void set_site(int leg, float x, float y, float z);
void wait_all_reach(void);
void cartesian_to_polar(volatile float &alpha, volatile float &beta, volatile float &gamma,
                        volatile float x, volatile float y, volatile float z);
void polar_to_servo(int leg, float alpha, float beta, float gamma);
void servo_attach(void);
void stand(void);
void sit(void);
void gait_step(float bx, float by);
void poll(void);
float legX(int leg, float bx, int phase);
float legY(int leg, float by, int phase);

void setup()
{
  LINK.begin(LINK_BAUD);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  for (int leg = 0; leg < 4; leg++)
  {
    int phase = (leg == 2) ? -1 : (leg == 3 ? +1 : 0);
    set_site(leg, legX(leg, 0, 0), legY(leg, 1, phase), z_boot);
  }

  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 3; j++)
      site_now[i][j] = site_expect[i][j];

  FlexiTimer2::set(20, servo_service);
  FlexiTimer2::start();

  servo_attach();
  delay(500);
  stand();
  delay(500);

  last_cmd_ms = millis();
}

void servo_attach(void)
{
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 3; j++)
    {
      servo[i][j].attach(servo_pin[i][j]);
      delay(100);
    }
}

void servo_detach(void)
{
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 3; j++)
    {
      servo[i][j].detach();
      delay(100);
    }
}

void loop()
{
  poll();
  update_sonar();

  float bx = cmd_bx, by = cmd_by;

  if (bx == 0 && by == 0) return;

  if (by > 0 && distance_cm >= 0 && distance_cm < OBSTACLE_CM) return;

  gait_step(bx, by);
}

float legX(int leg, float bx, int phase)
{
  return x_default + x_offset + phase * bx * x_amp * xSign[leg];
}

float legY(int leg, float by, int phase)
{
  return y_neutral + phase * by * y_amp * ySign[leg];
}

void swing_leg(int leg, float bx, float by)
{
  move_speed = leg_move_speed;
  set_site(leg, KEEP, KEEP, z_up);
  wait_all_reach();
  set_site(leg, legX(leg, bx, +1), legY(leg, by, +1), z_up);
  wait_all_reach();
  set_site(leg, KEEP, KEEP, z_default);
  wait_all_reach();
}

void body_shift(float bx, float by, bool phA)
{
  int phase[4];
  if (phA) { phase[0] = -1; phase[1] = -1; phase[2] =  0; phase[3] =  0; }
  else     { phase[0] =  0; phase[1] =  0; phase[2] = -1; phase[3] = -1; }

  move_speed = body_move_speed;
  for (int leg = 0; leg < 4; leg++)
    set_site(leg, legX(leg, bx, phase[leg]), legY(leg, by, phase[leg]), z_default);
  wait_all_reach();
  move_speed = leg_move_speed;
}

void resync(float bx, float by)
{
  int phase[4];
  if (phaseA) { phase[0] =  0; phase[1] =  0; phase[2] = -1; phase[3] = +1; }
  else        { phase[0] = -1; phase[1] = +1; phase[2] =  0; phase[3] =  0; }

  for (int leg = 0; leg < 4; leg++)
  {
    float tx = legX(leg, bx, phase[leg]);
    float ty = legY(leg, by, phase[leg]);
    if (fabs(tx - site_now[leg][0]) < 2.0 && fabs(ty - site_now[leg][1]) < 2.0) continue;

    move_speed = leg_move_speed;
    set_site(leg, KEEP, KEEP, z_up);
    wait_all_reach();
    set_site(leg, tx, ty, z_up);
    wait_all_reach();
    set_site(leg, KEEP, KEEP, z_default);
    wait_all_reach();
  }
}

void gait_step(float bx, float by)
{
  TRACE("step-begin");

  if (bx != cur_bx || by != cur_by)
  {
    TRACE("resync");
    resync(bx, by);
    cur_bx = bx;
    cur_by = by;
    TRACE("resync-done");
  }

  if (phaseA)
  {
    TRACE("swing2");
    swing_leg(2, bx, by);
    TRACE("body");
    body_shift(bx, by, true);
    TRACE("swing1");
    swing_leg(1, bx, by);
  }
  else
  {
    TRACE("swing0");
    swing_leg(0, bx, by);
    TRACE("body");
    body_shift(bx, by, false);
    TRACE("swing3");
    swing_leg(3, bx, by);
  }
  phaseA = !phaseA;
  TRACE("step-done");
}

void stand(void)
{
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++) set_site(leg, KEEP, KEEP, z_default);
  wait_all_reach();
}

void sit(void)
{
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++) set_site(leg, KEEP, KEEP, z_boot);
  wait_all_reach();
}

void read_commands(void)
{
  while (LINK.available())
  {
    char c = LINK.read();
    bool ok = true;
    switch (c)
    {
      case 'F': cmd_bx =  0.00; cmd_by =  1.00; break;
      case 'B': cmd_bx =  0.00; cmd_by = -1.00; break;
      case 'L': cmd_bx = -1.00; cmd_by =  0.00; break;
      case 'R': cmd_bx =  1.00; cmd_by =  0.00; break;
      case 'Q': cmd_bx = -0.70; cmd_by =  0.70; break;
      case 'E': cmd_bx =  0.70; cmd_by =  0.70; break;
      case 'Z': cmd_bx = -0.70; cmd_by = -0.70; break;
      case 'C': cmd_bx =  0.70; cmd_by = -0.70; break;
      case 'S': cmd_bx =  0.00; cmd_by =  0.00; break;
      default:  ok = false; break;
    }
    if (ok) {
      last_cmd_ms = millis();
#if DEBUG_TRACE
      LINK.print(F("#cmd:")); LINK.println(c);
#endif
    }
  }

  if (millis() - last_cmd_ms > CMD_TIMEOUT) { cmd_bx = 0; cmd_by = 0; }
}

void update_sonar(void)
{
  static unsigned long last = 0;
  static long hist[3] = { -1, -1, -1 };
  static byte idx = 0;

  if (millis() - last < PING_PERIOD) return;
  last = millis();

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long echo = pulseIn(ECHO_PIN, HIGH, 8000UL);
  long cm = (echo == 0) ? -1 : (long)(echo * 0.01715);
  if (cm < 2 || cm > 400) cm = -1;

  hist[idx] = cm;
  idx = (idx + 1) % 3;

  long a = hist[0], b = hist[1], c = hist[2];
  long m;
  if ((a <= b && b <= c) || (c <= b && b <= a))      m = b;
  else if ((b <= a && a <= c) || (c <= a && a <= b)) m = a;
  else                                               m = c;

  distance_cm = m;
}

void send_telemetry(void)
{
  static unsigned long last = 0;
  if (millis() - last < TX_PERIOD) return;
  last = millis();

  LINK.print('D');
  LINK.println(distance_cm);
}

void poll(void)
{
  read_commands();
  send_telemetry();
}

void servo_service(void)
{
  sei();
  static float alpha, beta, gamma;

  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      if (abs(site_now[i][j] - site_expect[i][j]) >= abs(temp_speed[i][j]))
        site_now[i][j] += temp_speed[i][j];
      else
        site_now[i][j] = site_expect[i][j];
    }

    cartesian_to_polar(alpha, beta, gamma, site_now[i][0], site_now[i][1], site_now[i][2]);
    polar_to_servo(i, alpha, beta, gamma);
  }
}

void set_site(int leg, float x, float y, float z)
{
  float length_x = 0, length_y = 0, length_z = 0;

  if (x != KEEP) length_x = x - site_now[leg][0];
  if (y != KEEP) length_y = y - site_now[leg][1];
  if (z != KEEP) length_z = z - site_now[leg][2];

  float length = sqrt(length_x * length_x + length_y * length_y + length_z * length_z);

  if (length < 0.01)
  {
    temp_speed[leg][0] = temp_speed[leg][1] = temp_speed[leg][2] = 0;
  }
  else
  {
    temp_speed[leg][0] = length_x / length * move_speed * speed_multiple;
    temp_speed[leg][1] = length_y / length * move_speed * speed_multiple;
    temp_speed[leg][2] = length_z / length * move_speed * speed_multiple;
  }

  if (x != KEEP) site_expect[leg][0] = x;
  if (y != KEEP) site_expect[leg][1] = y;
  if (z != KEEP) site_expect[leg][2] = z;
}

void wait_all_reach(void)
{
  bool done = false;
  while (!done)
  {
    poll();
    done = true;
    for (int i = 0; i < 4 && done; i++)
      for (int j = 0; j < 3; j++)
        if (site_now[i][j] != site_expect[i][j]) { done = false; break; }
  }
}

void cartesian_to_polar(volatile float &alpha, volatile float &beta, volatile float &gamma,
                        volatile float x, volatile float y, volatile float z)
{
  float v, w;
  w = (x >= 0 ? 1 : -1) * (sqrt(pow(x, 2) + pow(y, 2)));
  v = w - length_c;
  alpha = atan2(z, v) + acos((pow(length_a, 2) - pow(length_b, 2) + pow(v, 2) + pow(z, 2))
                             / 2 / length_a / sqrt(pow(v, 2) + pow(z, 2)));
  beta  = acos((pow(length_a, 2) + pow(length_b, 2) - pow(v, 2) - pow(z, 2))
                / 2 / length_a / length_b);
  gamma = (w >= 0) ? atan2(y, x) : atan2(-y, -x);

  alpha = alpha / pi * 180;
  beta  = beta  / pi * 180;
  gamma = gamma / pi * 180;
}

void polar_to_servo(int leg, float alpha, float beta, float gamma)
{
  if (leg == 0)
  {
    alpha = 90 - alpha;
    beta  = beta;
    gamma += 90;
  }
  else if (leg == 1)
  {
    alpha += 90;
    beta  = 180 - beta;
    gamma = 90 - gamma;
  }
  else if (leg == 2)
  {
    alpha += 90;
    beta  = 180 - beta;
    gamma = 90 - gamma;
  }
  else if (leg == 3)
  {
    alpha = 90 - alpha;
    beta  = beta;
    gamma += 90;
  }

  servo[leg][0].write(alpha);
  servo[leg][1].write(beta);
  servo[leg][2].write(gamma);
}