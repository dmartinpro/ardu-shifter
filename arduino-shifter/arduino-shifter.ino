unsigned int led_port = LED_BUILTIN; // Debug LED: the one on the Arduino board
unsigned int shifter_port = 2; // Arduino port for shifter interrupt
unsigned int debug_input_port = 4; // Debug digital input port: HIGH == DEBUG
unsigned int clutch_port = 5; // Arduino port for clutch switch
unsigned int ignition_port = 7; // Arduino port to trigger the transistor in order to enable/disable the ignition

unsigned int waiting_time_before_adj_port = 4; // Waiting time adjustment analog input port
unsigned int switchoff_time_adj_port = 5; // Switch off time adjustment analog input port
unsigned int waiting_time_after_adj_port = 3; // Waiting time adjustment analog input port

volatile boolean shifting = false; // true means that a gear change sequence is underway

volatile unsigned long switch_detected_time = 0; // Clock time when the event occurs
unsigned long waiting_time = 0; // How long (ms) between shift detection and ignition switch off
unsigned long shifting_time = 0; // How long (ms) the shift is supposed to last (ignition OFF)
unsigned long waiting_time_after = 0; // How long (ms) before accepting a new shift signal
unsigned long last_switch_on_time = 0;
unsigned long debug_offset = 1; // An offset value to slow down things when debugging (=easier to see the phases of the sequence)

boolean debug_enabled = false;

// a few constants
const unsigned int RIDING_PHASE = 0;
const unsigned int WAITING_PHASE = 1;
const unsigned int SHIFTING_PHASE = 2;
const unsigned int POST_SHIFTING_PHASE = 3;
const unsigned int MANUAL_PHASE = 4;

const unsigned long MIN_WAITING_TIME = 10; // ms
const unsigned long MAX_WAITING_TIME = 500; // ms
const unsigned long MIN_SHIFTING_TIME = 50; // ms
const unsigned long MAX_SHIFTING_TIME = 500; // ms
const unsigned long MIN_WAITING_TIME_AFTER = 10; // ms
const unsigned long MAX_WAITING_TIME_AFTER = 500; // ms

const unsigned long DEBUG_OFFSET = 10; // The constant applied when debugging, see the variable 'debug_offset'

void setup() {
  Serial.begin(9600);
  Serial.println("Initializing");

  // Digital output
  pinMode(led_port, OUTPUT);
  pinMode(ignition_port, OUTPUT);
  // Digital input
  pinMode(clutch_port, INPUT);
  pinMode(shifter_port, INPUT);
  pinMode(debug_input_port, INPUT);
  
  // Analog input
  pinMode(waiting_time_before_adj_port, INPUT);
  pinMode(switchoff_time_adj_port, INPUT);
  pinMode(waiting_time_after_adj_port, INPUT);

  // Initialize outputs to the right values
  digitalWrite(led_port, (debug_enabled ? HIGH : LOW));
  digitalWrite(ignition_port, HIGH);

  attachInterrupt(shifter_port-2, shifter_triggered, RISING); // when the signal goes UP (switch is open)
  Serial.println("Initialized");
}

/**
 * What is supposed to happen here:
 *
 * In every loop :
 * - check if debug mode is activated (a switch)
 * - adjust the waiting time (a potentiometer)
 * - adjust the shiting time (a potentiometer)
 *
 * Then check if a shift order was detected and is currently processed
 * (ie: shifting == true) with 'shifting' variable
 * If so, try to determine which part of the sequence the process should be in:
 * - The waiting phase, aka phase 1, before switching off the ignition, which lasts a few milliseconds (adjustable)
 * - The ignition switch off phase, aka phase 2, which last a few milliseconds (adjustable too)
 * - The last phase, aka phase 3, where the ignition is switch back on and the current state is reverted to 'normal'
 * - And a particular phase, aka phase 4, where the clutch lever is used and the quickshifter shouldn't do anything
 *
 */
void loop() {
  debug();
  set_adjustable_values();
  if (debug_enabled) {
    delay(debug_offset);
  }

  if (shifting) {
    int current_phase = get_phase();

    switch (current_phase) {
      case MANUAL_PHASE:
        do_manual();
        break;
      case WAITING_PHASE:
        do_wait();
        break;
      case SHIFTING_PHASE:
        do_shift();
        break;
      case POST_SHIFTING_PHASE:
        do_post_shift();
        break;
      case RIDING_PHASE:
        do_ride();
        break;
    }
 
  }

}

/**
 * Function called whenever the shifter switch event (see attachInterrupt for the event type) is raised
 */
void shifter_triggered() {
  if (!shifting && (millis()-last_switch_on_time > waiting_time_after) ) { // meaning, no previous event currently processed AND not to close to the previous switch event
    shifting = true;
    switch_detected_time = millis();
    if (debug_enabled) {
      Serial.println("Shift event order received");
    }
  } else if (debug_enabled) {
    Serial.println("Shift event ignored: already processing one");
  }
}

int get_phase() {
  if (switch_detected_time == 0 || !shifting) { // No shift order detected, we are riding
    return RIDING_PHASE;
  }

  boolean clutch_in_use = digitalRead(clutch_port);
  if (clutch_in_use) {
    return MANUAL_PHASE;
  }

  long current = millis();
  long elapsed = (current - switch_detected_time); // TODO: be careful when clock is cycling (every 40 days)

  if (elapsed <= (waiting_time * debug_offset)) {
    return WAITING_PHASE;
  } else if (elapsed <= ( (waiting_time+shifting_time) * debug_offset)) {
    return SHIFTING_PHASE;
  } else if (elapsed > ( (waiting_time+shifting_time) * debug_offset)) {
    return POST_SHIFTING_PHASE;
  }

  return POST_SHIFTING_PHASE;
}

void do_manual() {
  if (debug_enabled) {
    Serial.println("Manual mode...");
  }
  set_riding_state();
}

void do_ride() {
  if (debug_enabled) {
    Serial.println("Riding...");
  }
}

void do_wait() {
  if (debug_enabled) {
//    Serial.println("Waiting...");
  }
}

/**
 * Shift means switch off the ignition.
 * Before switching it off, just check its current state : it may already be switch off due to a previous cycle
 */
void do_shift() {

  int on = digitalRead(ignition_port);
  if (on) { // ok, switch it off
    if (debug_enabled) {
      Serial.println("Shifting (switch off ignition)...");
      long elapsed = (millis() - switch_detected_time);
      Serial.print(elapsed);
      Serial.println(" ms");
    }
    digitalWrite(ignition_port, 0);
  }

}

/**
 * Post shift means switch on the ignition.
 * Before switching it on, just check its current state : it may already be switch on due to a previous cycle
 */
void do_post_shift() {
  int on = digitalRead(ignition_port);
  if (!on) { // ok, switch it on
    digitalWrite(ignition_port, 1);
    last_switch_on_time = millis();
    if (debug_enabled) {
      Serial.println("Post shifting: ignition is back...");
      long elapsed = (millis() - switch_detected_time);
      Serial.print(elapsed);
      Serial.println(" ms");
    }
  }
  set_riding_state();
}

/**
 * Reset the variables to the nominal values (gear engaged)
 */
void set_riding_state() {
  shifting = false;
  switch_detected_time = 0;
}

/**
 * Read the analog input and determine the adjustable parameters based on them
 */
void set_adjustable_values() {
  int raw_waiting_value = analogRead(waiting_time_before_adj_port); // 0-1023
  int raw_switchingoff_value = analogRead(switchoff_time_adj_port); // 0-1023
  int raw_waiting_after_value = analogRead(waiting_time_after_adj_port); // 0-1023

  long prev_waiting_time = waiting_time;
  long prev_shifting_time = shifting_time;
  long prev_waiting_time_after = waiting_time_after;

  waiting_time = MIN_WAITING_TIME + (MAX_WAITING_TIME-MIN_WAITING_TIME)*(float)raw_waiting_value/1023;
  shifting_time = MIN_SHIFTING_TIME + (MAX_SHIFTING_TIME-MIN_SHIFTING_TIME)*(float)raw_switchingoff_value/1023;
  waiting_time_after = MIN_WAITING_TIME_AFTER + (MAX_WAITING_TIME_AFTER-MIN_WAITING_TIME_AFTER)*(float)raw_waiting_after_value/1023;
  waiting_time_after = waiting_time; // TODO: If a specific adj. resistor is plugged to PIN6, delete this line. Just a quick hack.

  if (debug_enabled) {
    if (prev_waiting_time < waiting_time*0.9 || prev_waiting_time > waiting_time*1.1) {
      Serial.print("New waiting_time=");
      Serial.println(waiting_time);
    }
    if (prev_shifting_time < shifting_time*0.9 || prev_shifting_time > shifting_time*1.1) {
      Serial.print("New shifting_time=");
      Serial.println(shifting_time);
    }
    if (prev_waiting_time_after < waiting_time_after*0.9 || prev_waiting_time_after > waiting_time_after*1.1) {
      Serial.print("New waiting_time_after=");
      Serial.println(waiting_time_after);
    }
  }

}

/**
 * Enable/disable debug mode
 */
void debug() {
  boolean previous_state = debug_enabled;
  debug_enabled = digitalRead(debug_input_port);
  if (debug_enabled) {
    debug_offset = DEBUG_OFFSET;
  } else {
    debug_offset = 1;
  }
  if (previous_state != debug_enabled) {
    digitalWrite(led_port, (debug_enabled ? HIGH : LOW));
    Serial.print("DEBUG mode is now: ");
    Serial.println(debug_enabled ? "ON" : "OFF");
    Serial.print("debug_offset=");
    Serial.println(debug_offset);
  }
}
