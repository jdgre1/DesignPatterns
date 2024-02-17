#pragma once

#ifndef UTILS_H
#define UTILS_H

#include <stdio.h>
#include <stdint.h>

namespace patterns
{

extern size_t FIELD_HEIGHT_MAX;
extern size_t FIELD_WIDTH_MAX;
using Entity = uint32_t; // An entity is just an id

const Entity MAX_NUM_ENTITIES = 100;

} // namespace patterns

#endif  // UTILS_H
