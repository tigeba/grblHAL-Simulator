/*
  grbl_interface.c - hooks to interact with grbl app running on simulator

  Part of Grbl Simulator

  Copyright (c) 2012 Jens Geisler
  Copyright (c) 2014-2015 Adam Shelly

  2020-2025 - modified for grblHAL by Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>

#include "mcu.h"
#include "driver.h"
#include "simulator.h"

#include "grbl/hal.h"
#include "grbl/protocol.h"
#include "grbl/state_machine.h"
#include "grbl/settings.h"
#include "grbl/probe.h"

int block_position[N_AXIS] = {0}; //step count after most recently planned block
uint32_t block_number = 0;
double next_print_time;

// Simulated toolsetter surface height in machine coordinates (mm).
// When probing, the probe pin triggers when Z reaches this height.
// Set to 0.0 to simulate a toolsetter at the Z machine origin.
// Configurable via TOOLSETTER_Z_MM environment variable.
static float toolsetter_z_mm = -50.0f;
static bool toolsetter_initialized = false;
static bool probe_pin_active = false;

static void print_steps(bool force);
static void printBlock(void);
static void simulate_probe(void);
static void simulate_limits(void);

void grbl_app_init (void)
{
    //setup local tacking vars
    next_print_time = args.step_time;

    // Allow toolsetter height to be configured via environment variable
    const char *env_z = getenv("TOOLSETTER_Z_MM");
    if (env_z) {
        toolsetter_z_mm = (float)atof(env_z);
        fprintf(stderr, "[SIM] Toolsetter Z surface: %.3f mm (from env)\n", toolsetter_z_mm);
    } else {
        fprintf(stderr, "[SIM] Toolsetter Z surface: %.3f mm (default)\n", toolsetter_z_mm);
    }
    toolsetter_initialized = true;
}

void grbl_per_tick (void)
{
    //maybe print the position every tick
    print_steps(0);

    // Simulate probe pin (toolsetter) based on Z position
    simulate_probe();

    // Simulate limit switches based on axis positions
    simulate_limits();
}

// Simulate toolsetter: trigger probe pin when Z reaches the toolsetter surface
static void simulate_probe (void)
{
    if (!toolsetter_initialized)
        return;

    if (sys.probing_state == Probing_Active) {
        // Convert current Z position from steps to mm
        float z_mm = (float)sys.position[Z_AXIS] / settings.axis[Z_AXIS].steps_per_mm;

        // Trigger probe when Z descends to (or past) the toolsetter surface
        if (z_mm <= toolsetter_z_mm && !probe_pin_active) {
            mcu_gpio_in(&gpio[PROBE_PORT], PROBE_BIT, PROBE_BIT);  // set probe triggered
            probe_pin_active = true;
            fprintf(stderr, "[SIM] Probe triggered at Z=%.3f mm\n", z_mm);
        }
    } else {
        // Clear probe pin when not actively probing
        if (probe_pin_active) {
            mcu_gpio_in(&gpio[PROBE_PORT], 0, PROBE_BIT);  // clear probe triggered
            probe_pin_active = false;
        }
    }
}

// Simulate limit switches based on axis travel limits
static void simulate_limits (void)
{
    // Trigger limit switches when axes exceed max travel (from $130/$131/$132)
    // This enables simulated homing and soft limit behavior.
    float pos[N_AXIS];
    system_convert_array_steps_to_mpos(pos, sys.position);

    uint16_t limits = 0;

    if (pos[X_AXIS] <= -settings.axis[X_AXIS].max_travel || pos[X_AXIS] >= 0.0f)
        limits |= X_AXIS_BIT;
    if (pos[Y_AXIS] <= -settings.axis[Y_AXIS].max_travel || pos[Y_AXIS] >= 0.0f)
        limits |= Y_AXIS_BIT;
    if (pos[Z_AXIS] <= -settings.axis[Z_AXIS].max_travel || pos[Z_AXIS] >= 0.0f)
        limits |= Z_AXIS_BIT;

    mcu_gpio_in(&gpio[LIMITS_PORT0], limits, X_AXIS_BIT | Y_AXIS_BIT | Z_AXIS_BIT);
}

void grbl_per_byte (void)
{
    if(sim.socket_fd) {
        switch (platform_poll_stdin()) {

            case 'e':
            case 'E':
                mcu_gpio_toggle_in(&gpio[CONTROL_PORT], ESTOP_BIT);
                break;

            case 'r':
            case 'R':
                mcu_gpio_toggle_in(&gpio[CONTROL_PORT], RESET_BIT);
                break;

            case 'h':
            case 'H':
                mcu_gpio_toggle_in(&gpio[CONTROL_PORT], FEED_HOLD_BIT);
                break;

            case 's':
            case 'S':
                mcu_gpio_toggle_in(&gpio[CONTROL_PORT], CYCLE_START_BIT);
                break;

            case 'd':
            case 'D':
                mcu_gpio_toggle_in(&gpio[CONTROL_PORT], SAFETY_DOOR_BIT);
                break;

            case 'p':
            case 'P':
                mcu_gpio_toggle_in(&gpio[PROBE_PORT], PROBE_BIT);
                break;

            case 'o':
            case 'O':
                mcu_gpio_toggle_in(&gpio[PROBE_PORT], PROBE_CONNECTED_BIT);
                break;

            case 'x':
                mcu_gpio_toggle_in(&gpio[LIMITS_PORT0], X_AXIS_BIT);
                break;

            case 'y':
                mcu_gpio_toggle_in(&gpio[LIMITS_PORT0], Y_AXIS_BIT);
                break;

            case 'z':
                mcu_gpio_toggle_in(&gpio[LIMITS_PORT0], Z_AXIS_BIT);
                break;

            case 'X':
                mcu_gpio_toggle_in(&gpio[LIMITS_PORT1], X_AXIS_BIT);
                break;

            case 'Y':
                mcu_gpio_toggle_in(&gpio[LIMITS_PORT1], Y_AXIS_BIT);
                break;

            case 'Z':
                mcu_gpio_toggle_in(&gpio[LIMITS_PORT1], Z_AXIS_BIT);
                break;

            case '?':
                protocol_enqueue_realtime_command(CMD_STATUS_REPORT);
                break;

            case '@':
                protocol_enqueue_realtime_command(CMD_STATUS_REPORT_ALL);
                break;

            case 0x06:
                sim.exit = exit_REQ;
                break;
        }

//        if(args.block_out_file != stdout)
            printBlock();   //maybe print newest block
    } else
        printBlock();   //maybe print newest block
}

void grbl_app_exit (void)
{
    //force final position print
    print_steps(1);
}

//show current position in steps
static void print_steps (bool force)
{ 
    static plan_block_t* printed_block = NULL;

    plan_block_t* current_block = plan_get_current_block();
    int ocr = 0;

    //Allow exit when idle. Prevents aborting before all streamed commands have run
    if (sim.exit == exit_REQ && state_get() < STATE_HOMING )
        sim.exit = exit_OK;

    if (next_print_time == 0.0)
        return;  //no printing

    #ifdef VARIABLE_SPINDLE
    if(SPINDLE_TCCRA_REGISTER >= 127) ocr = SPINDLE_OCR_REGISTER;
    #endif

    if (current_block != printed_block) {
        //new block. 
        if (block_number) //print values from the end of prev block
            fprintf(args.step_out_file, "%12.5f %d, %d, %d, %d\n", sim.sim_time, sys.position[X_AXIS], sys.position[Y_AXIS], sys.position[Z_AXIS],ocr);

        printed_block = current_block;
        if (current_block == NULL)
            return;
        // print header
        fprintf(args.step_out_file, "# block number %d\n", block_number++);
    }
    //print at correct interval while executing block
    else if ((current_block && sim.sim_time>=next_print_time) || force ) {
        fprintf(args.step_out_file, "%12.5f %d, %d, %d, %d\n", sim.sim_time, sys.position[X_AXIS], sys.position[Y_AXIS], sys.position[Z_AXIS], ocr);
        fflush(args.step_out_file);
        //make sure the simulation time doesn't get ahead of next_print_time
        while (next_print_time <= sim.sim_time)
            next_print_time += args.step_time;
    }
}

// Print information about the most recently inserted block
// but only once!
static void printBlock (void)
{
    static plan_block_t *last_block;

    plan_block_t *b = plan_get_recent_block();
    if(b != last_block && b != NULL) {
        int i;
        for (i = 0; i < N_AXIS; i++) {
            if(b->direction.bits & bit(i))
                block_position[i] -= b->steps.value[i];
            else
                block_position[i] += b->steps.value[i];
            fprintf(args.block_out_file,"%d, ", block_position[i]);
        }
        fprintf(args.block_out_file,"%f\n", b->entry_speed_sqr);
        fflush(args.block_out_file); //TODO: needed?
        last_block = b;
    }
}
