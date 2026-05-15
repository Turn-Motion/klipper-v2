// Simulated GPIO for Linux MCU: no hardware access, all state in memory.
// Used by sim/entrypoint.sh during Docker simulation builds.
//
// Copyright (C) 2024-2026  Klipper contributors
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdint.h>
#include <string.h>
#include "command.h"     // shutdown, DECL_ENUMERATION_RANGE
#include "gpio.h"        // gpio_out, gpio_in structs
#include "internal.h"    // GPIO(), MAX_GPIO_LINES, GPIO2PIN, GPIO2PORT
#include "sched.h"       // sched_shutdown

// ---------------------------------------------------------------------------
// Pin enumeration matches gpio.c so the same pin names work.
// ---------------------------------------------------------------------------
DECL_ENUMERATION_RANGE("pin", "gpio0",         GPIO(0, 0), MAX_GPIO_LINES);
DECL_ENUMERATION_RANGE("pin", "gpiochip0/gpio0", GPIO(0, 0), MAX_GPIO_LINES);
DECL_ENUMERATION_RANGE("pin", "gpiochip1/gpio0", GPIO(1, 0), MAX_GPIO_LINES);
DECL_ENUMERATION_RANGE("pin", "gpiochip2/gpio0", GPIO(2, 0), MAX_GPIO_LINES);
DECL_ENUMERATION_RANGE("pin", "gpiochip3/gpio0", GPIO(3, 0), MAX_GPIO_LINES);
DECL_ENUMERATION_RANGE("pin", "gpiochip4/gpio0", GPIO(4, 0), MAX_GPIO_LINES);
DECL_ENUMERATION_RANGE("pin", "gpiochip5/gpio0", GPIO(5, 0), MAX_GPIO_LINES);
DECL_ENUMERATION_RANGE("pin", "gpiochip6/gpio0", GPIO(6, 0), MAX_GPIO_LINES);
DECL_ENUMERATION_RANGE("pin", "gpiochip7/gpio0", GPIO(7, 0), MAX_GPIO_LINES);
DECL_ENUMERATION_RANGE("pin", "gpiochip8/gpio0", GPIO(8, 0), MAX_GPIO_LINES);

// ---------------------------------------------------------------------------
// gpio_line and state storage
// ---------------------------------------------------------------------------
#define TOTAL_PINS (9 * MAX_GPIO_LINES)

struct gpio_line {
    uint32_t pin;
};
static struct gpio_line gpio_pin_store[TOTAL_PINS];
static uint8_t out_state[TOTAL_PINS];
static uint8_t in_state[TOTAL_PINS];

// ---------------------------------------------------------------------------
// gpio_out: output pins (steppers/heaters/fans)
// ---------------------------------------------------------------------------
struct gpio_out
gpio_out_setup(uint32_t pin, uint8_t val)
{
    if (pin >= TOTAL_PINS)
        shutdown("gpio_out_setup: pin out of range");
    gpio_pin_store[pin].pin = pin;
    out_state[pin] = !!val;
    struct gpio_out g = { .line = &gpio_pin_store[pin] };
    return g;
}

void
gpio_out_reset(struct gpio_out g, uint8_t val)
{
    uint32_t pin = g.line->pin;
    if (pin < TOTAL_PINS)
        out_state[pin] = !!val;
}

void
gpio_out_write(struct gpio_out g, uint8_t val)
{
    uint32_t pin = g.line->pin;
    if (pin < TOTAL_PINS)
        out_state[pin] = !!val;
}

void
gpio_out_toggle_noirq(struct gpio_out g)
{
    uint32_t pin = g.line->pin;
    if (pin < TOTAL_PINS)
        out_state[pin] ^= 1;
}

void
gpio_out_toggle(struct gpio_out g)
{
    gpio_out_toggle_noirq(g);
}

// ---------------------------------------------------------------------------
// gpio_in: input pins (endstops/probes).
// Return triggered in simulation so homing sequences complete.
// ---------------------------------------------------------------------------
struct gpio_in
gpio_in_setup(uint32_t pin, int8_t pull_up)
{
    if (pin >= TOTAL_PINS)
        shutdown("gpio_in_setup: pin out of range");
    gpio_pin_store[pin].pin = pin;
    in_state[pin] = 1;
    struct gpio_in g = { .line = &gpio_pin_store[pin] };
    return g;
}

void
gpio_in_reset(struct gpio_in g, int8_t pull_up)
{
    uint32_t pin = g.line->pin;
    if (pin < TOTAL_PINS)
        in_state[pin] = 1;
}

uint8_t
gpio_in_read(struct gpio_in g)
{
    uint32_t pin = g.line->pin;
    if (pin < TOTAL_PINS)
        return in_state[pin];
    return 1;
}
