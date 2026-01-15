#ifndef LIGHTS_H
#define LIGHTS_H

#include <stdbool.h>

void lights_init(void);
void lights_set_left(bool on);
void lights_set_right(bool on);
void lights_toggle_left(void);
void lights_toggle_right(void);

#endif // LIGHTS_H
