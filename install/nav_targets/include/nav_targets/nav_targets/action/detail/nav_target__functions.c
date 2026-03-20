// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from nav_targets:action/NavTarget.idl
// generated code does not contain a copyright notice
#include "nav_targets/action/detail/nav_target__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `goal`
#include "geometry_msgs/msg/detail/point_stamped__functions.h"

bool
nav_targets__action__NavTarget_Goal__init(nav_targets__action__NavTarget_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // goal
  if (!geometry_msgs__msg__PointStamped__init(&msg->goal)) {
    nav_targets__action__NavTarget_Goal__fini(msg);
    return false;
  }
  return true;
}

void
nav_targets__action__NavTarget_Goal__fini(nav_targets__action__NavTarget_Goal * msg)
{
  if (!msg) {
    return;
  }
  // goal
  geometry_msgs__msg__PointStamped__fini(&msg->goal);
}

bool
nav_targets__action__NavTarget_Goal__are_equal(const nav_targets__action__NavTarget_Goal * lhs, const nav_targets__action__NavTarget_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal
  if (!geometry_msgs__msg__PointStamped__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
nav_targets__action__NavTarget_Goal__copy(
  const nav_targets__action__NavTarget_Goal * input,
  nav_targets__action__NavTarget_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // goal
  if (!geometry_msgs__msg__PointStamped__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

nav_targets__action__NavTarget_Goal *
nav_targets__action__NavTarget_Goal__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nav_targets__action__NavTarget_Goal * msg = (nav_targets__action__NavTarget_Goal *)allocator.allocate(sizeof(nav_targets__action__NavTarget_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(nav_targets__action__NavTarget_Goal));
  bool success = nav_targets__action__NavTarget_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
nav_targets__action__NavTarget_Goal__destroy(nav_targets__action__NavTarget_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    nav_targets__action__NavTarget_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
nav_targets__action__NavTarget_Goal__Sequence__init(nav_targets__action__NavTarget_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nav_targets__action__NavTarget_Goal * data = NULL;

  if (size) {
    data = (nav_targets__action__NavTarget_Goal *)allocator.zero_allocate(size, sizeof(nav_targets__action__NavTarget_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = nav_targets__action__NavTarget_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        nav_targets__action__NavTarget_Goal__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
nav_targets__action__NavTarget_Goal__Sequence__fini(nav_targets__action__NavTarget_Goal__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      nav_targets__action__NavTarget_Goal__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

nav_targets__action__NavTarget_Goal__Sequence *
nav_targets__action__NavTarget_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nav_targets__action__NavTarget_Goal__Sequence * array = (nav_targets__action__NavTarget_Goal__Sequence *)allocator.allocate(sizeof(nav_targets__action__NavTarget_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = nav_targets__action__NavTarget_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
nav_targets__action__NavTarget_Goal__Sequence__destroy(nav_targets__action__NavTarget_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    nav_targets__action__NavTarget_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
nav_targets__action__NavTarget_Goal__Sequence__are_equal(const nav_targets__action__NavTarget_Goal__Sequence * lhs, const nav_targets__action__NavTarget_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!nav_targets__action__NavTarget_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
nav_targets__action__NavTarget_Goal__Sequence__copy(
  const nav_targets__action__NavTarget_Goal__Sequence * input,
  nav_targets__action__NavTarget_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(nav_targets__action__NavTarget_Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    nav_targets__action__NavTarget_Goal * data =
      (nav_targets__action__NavTarget_Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!nav_targets__action__NavTarget_Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          nav_targets__action__NavTarget_Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!nav_targets__action__NavTarget_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
nav_targets__action__NavTarget_Result__init(nav_targets__action__NavTarget_Result * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
nav_targets__action__NavTarget_Result__fini(nav_targets__action__NavTarget_Result * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
nav_targets__action__NavTarget_Result__are_equal(const nav_targets__action__NavTarget_Result * lhs, const nav_targets__action__NavTarget_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
nav_targets__action__NavTarget_Result__copy(
  const nav_targets__action__NavTarget_Result * input,
  nav_targets__action__NavTarget_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

nav_targets__action__NavTarget_Result *
nav_targets__action__NavTarget_Result__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nav_targets__action__NavTarget_Result * msg = (nav_targets__action__NavTarget_Result *)allocator.allocate(sizeof(nav_targets__action__NavTarget_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(nav_targets__action__NavTarget_Result));
  bool success = nav_targets__action__NavTarget_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
nav_targets__action__NavTarget_Result__destroy(nav_targets__action__NavTarget_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    nav_targets__action__NavTarget_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
nav_targets__action__NavTarget_Result__Sequence__init(nav_targets__action__NavTarget_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nav_targets__action__NavTarget_Result * data = NULL;

  if (size) {
    data = (nav_targets__action__NavTarget_Result *)allocator.zero_allocate(size, sizeof(nav_targets__action__NavTarget_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = nav_targets__action__NavTarget_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        nav_targets__action__NavTarget_Result__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
nav_targets__action__NavTarget_Result__Sequence__fini(nav_targets__action__NavTarget_Result__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      nav_targets__action__NavTarget_Result__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

nav_targets__action__NavTarget_Result__Sequence *
nav_targets__action__NavTarget_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nav_targets__action__NavTarget_Result__Sequence * array = (nav_targets__action__NavTarget_Result__Sequence *)allocator.allocate(sizeof(nav_targets__action__NavTarget_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = nav_targets__action__NavTarget_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
nav_targets__action__NavTarget_Result__Sequence__destroy(nav_targets__action__NavTarget_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    nav_targets__action__NavTarget_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
nav_targets__action__NavTarget_Result__Sequence__are_equal(const nav_targets__action__NavTarget_Result__Sequence * lhs, const nav_targets__action__NavTarget_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!nav_targets__action__NavTarget_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
nav_targets__action__NavTarget_Result__Sequence__copy(
  const nav_targets__action__NavTarget_Result__Sequence * input,
  nav_targets__action__NavTarget_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(nav_targets__action__NavTarget_Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    nav_targets__action__NavTarget_Result * data =
      (nav_targets__action__NavTarget_Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!nav_targets__action__NavTarget_Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          nav_targets__action__NavTarget_Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!nav_targets__action__NavTarget_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `distance`
#include "std_msgs/msg/detail/float32__functions.h"

bool
nav_targets__action__NavTarget_Feedback__init(nav_targets__action__NavTarget_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // distance
  if (!std_msgs__msg__Float32__init(&msg->distance)) {
    nav_targets__action__NavTarget_Feedback__fini(msg);
    return false;
  }
  return true;
}

void
nav_targets__action__NavTarget_Feedback__fini(nav_targets__action__NavTarget_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // distance
  std_msgs__msg__Float32__fini(&msg->distance);
}

bool
nav_targets__action__NavTarget_Feedback__are_equal(const nav_targets__action__NavTarget_Feedback * lhs, const nav_targets__action__NavTarget_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // distance
  if (!std_msgs__msg__Float32__are_equal(
      &(lhs->distance), &(rhs->distance)))
  {
    return false;
  }
  return true;
}

bool
nav_targets__action__NavTarget_Feedback__copy(
  const nav_targets__action__NavTarget_Feedback * input,
  nav_targets__action__NavTarget_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // distance
  if (!std_msgs__msg__Float32__copy(
      &(input->distance), &(output->distance)))
  {
    return false;
  }
  return true;
}

nav_targets__action__NavTarget_Feedback *
nav_targets__action__NavTarget_Feedback__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nav_targets__action__NavTarget_Feedback * msg = (nav_targets__action__NavTarget_Feedback *)allocator.allocate(sizeof(nav_targets__action__NavTarget_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(nav_targets__action__NavTarget_Feedback));
  bool success = nav_targets__action__NavTarget_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
nav_targets__action__NavTarget_Feedback__destroy(nav_targets__action__NavTarget_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    nav_targets__action__NavTarget_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
nav_targets__action__NavTarget_Feedback__Sequence__init(nav_targets__action__NavTarget_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nav_targets__action__NavTarget_Feedback * data = NULL;

  if (size) {
    data = (nav_targets__action__NavTarget_Feedback *)allocator.zero_allocate(size, sizeof(nav_targets__action__NavTarget_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = nav_targets__action__NavTarget_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        nav_targets__action__NavTarget_Feedback__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
nav_targets__action__NavTarget_Feedback__Sequence__fini(nav_targets__action__NavTarget_Feedback__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      nav_targets__action__NavTarget_Feedback__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

nav_targets__action__NavTarget_Feedback__Sequence *
nav_targets__action__NavTarget_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nav_targets__action__NavTarget_Feedback__Sequence * array = (nav_targets__action__NavTarget_Feedback__Sequence *)allocator.allocate(sizeof(nav_targets__action__NavTarget_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = nav_targets__action__NavTarget_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
nav_targets__action__NavTarget_Feedback__Sequence__destroy(nav_targets__action__NavTarget_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    nav_targets__action__NavTarget_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
nav_targets__action__NavTarget_Feedback__Sequence__are_equal(const nav_targets__action__NavTarget_Feedback__Sequence * lhs, const nav_targets__action__NavTarget_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!nav_targets__action__NavTarget_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
nav_targets__action__NavTarget_Feedback__Sequence__copy(
  const nav_targets__action__NavTarget_Feedback__Sequence * input,
  nav_targets__action__NavTarget_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(nav_targets__action__NavTarget_Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    nav_targets__action__NavTarget_Feedback * data =
      (nav_targets__action__NavTarget_Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!nav_targets__action__NavTarget_Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          nav_targets__action__NavTarget_Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!nav_targets__action__NavTarget_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "nav_targets/action/detail/nav_target__functions.h"

bool
nav_targets__action__NavTarget_SendGoal_Request__init(nav_targets__action__NavTarget_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    nav_targets__action__NavTarget_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!nav_targets__action__NavTarget_Goal__init(&msg->goal)) {
    nav_targets__action__NavTarget_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
nav_targets__action__NavTarget_SendGoal_Request__fini(nav_targets__action__NavTarget_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  nav_targets__action__NavTarget_Goal__fini(&msg->goal);
}

bool
nav_targets__action__NavTarget_SendGoal_Request__are_equal(const nav_targets__action__NavTarget_SendGoal_Request * lhs, const nav_targets__action__NavTarget_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!nav_targets__action__NavTarget_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
nav_targets__action__NavTarget_SendGoal_Request__copy(
  const nav_targets__action__NavTarget_SendGoal_Request * input,
  nav_targets__action__NavTarget_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!nav_targets__action__NavTarget_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

nav_targets__action__NavTarget_SendGoal_Request *
nav_targets__action__NavTarget_SendGoal_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nav_targets__action__NavTarget_SendGoal_Request * msg = (nav_targets__action__NavTarget_SendGoal_Request *)allocator.allocate(sizeof(nav_targets__action__NavTarget_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(nav_targets__action__NavTarget_SendGoal_Request));
  bool success = nav_targets__action__NavTarget_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
nav_targets__action__NavTarget_SendGoal_Request__destroy(nav_targets__action__NavTarget_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    nav_targets__action__NavTarget_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
nav_targets__action__NavTarget_SendGoal_Request__Sequence__init(nav_targets__action__NavTarget_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nav_targets__action__NavTarget_SendGoal_Request * data = NULL;

  if (size) {
    data = (nav_targets__action__NavTarget_SendGoal_Request *)allocator.zero_allocate(size, sizeof(nav_targets__action__NavTarget_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = nav_targets__action__NavTarget_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        nav_targets__action__NavTarget_SendGoal_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
nav_targets__action__NavTarget_SendGoal_Request__Sequence__fini(nav_targets__action__NavTarget_SendGoal_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      nav_targets__action__NavTarget_SendGoal_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

nav_targets__action__NavTarget_SendGoal_Request__Sequence *
nav_targets__action__NavTarget_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nav_targets__action__NavTarget_SendGoal_Request__Sequence * array = (nav_targets__action__NavTarget_SendGoal_Request__Sequence *)allocator.allocate(sizeof(nav_targets__action__NavTarget_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = nav_targets__action__NavTarget_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
nav_targets__action__NavTarget_SendGoal_Request__Sequence__destroy(nav_targets__action__NavTarget_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    nav_targets__action__NavTarget_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
nav_targets__action__NavTarget_SendGoal_Request__Sequence__are_equal(const nav_targets__action__NavTarget_SendGoal_Request__Sequence * lhs, const nav_targets__action__NavTarget_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!nav_targets__action__NavTarget_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
nav_targets__action__NavTarget_SendGoal_Request__Sequence__copy(
  const nav_targets__action__NavTarget_SendGoal_Request__Sequence * input,
  nav_targets__action__NavTarget_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(nav_targets__action__NavTarget_SendGoal_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    nav_targets__action__NavTarget_SendGoal_Request * data =
      (nav_targets__action__NavTarget_SendGoal_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!nav_targets__action__NavTarget_SendGoal_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          nav_targets__action__NavTarget_SendGoal_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!nav_targets__action__NavTarget_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
nav_targets__action__NavTarget_SendGoal_Response__init(nav_targets__action__NavTarget_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    nav_targets__action__NavTarget_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
nav_targets__action__NavTarget_SendGoal_Response__fini(nav_targets__action__NavTarget_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
nav_targets__action__NavTarget_SendGoal_Response__are_equal(const nav_targets__action__NavTarget_SendGoal_Response * lhs, const nav_targets__action__NavTarget_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
nav_targets__action__NavTarget_SendGoal_Response__copy(
  const nav_targets__action__NavTarget_SendGoal_Response * input,
  nav_targets__action__NavTarget_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

nav_targets__action__NavTarget_SendGoal_Response *
nav_targets__action__NavTarget_SendGoal_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nav_targets__action__NavTarget_SendGoal_Response * msg = (nav_targets__action__NavTarget_SendGoal_Response *)allocator.allocate(sizeof(nav_targets__action__NavTarget_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(nav_targets__action__NavTarget_SendGoal_Response));
  bool success = nav_targets__action__NavTarget_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
nav_targets__action__NavTarget_SendGoal_Response__destroy(nav_targets__action__NavTarget_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    nav_targets__action__NavTarget_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
nav_targets__action__NavTarget_SendGoal_Response__Sequence__init(nav_targets__action__NavTarget_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nav_targets__action__NavTarget_SendGoal_Response * data = NULL;

  if (size) {
    data = (nav_targets__action__NavTarget_SendGoal_Response *)allocator.zero_allocate(size, sizeof(nav_targets__action__NavTarget_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = nav_targets__action__NavTarget_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        nav_targets__action__NavTarget_SendGoal_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
nav_targets__action__NavTarget_SendGoal_Response__Sequence__fini(nav_targets__action__NavTarget_SendGoal_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      nav_targets__action__NavTarget_SendGoal_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

nav_targets__action__NavTarget_SendGoal_Response__Sequence *
nav_targets__action__NavTarget_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nav_targets__action__NavTarget_SendGoal_Response__Sequence * array = (nav_targets__action__NavTarget_SendGoal_Response__Sequence *)allocator.allocate(sizeof(nav_targets__action__NavTarget_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = nav_targets__action__NavTarget_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
nav_targets__action__NavTarget_SendGoal_Response__Sequence__destroy(nav_targets__action__NavTarget_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    nav_targets__action__NavTarget_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
nav_targets__action__NavTarget_SendGoal_Response__Sequence__are_equal(const nav_targets__action__NavTarget_SendGoal_Response__Sequence * lhs, const nav_targets__action__NavTarget_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!nav_targets__action__NavTarget_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
nav_targets__action__NavTarget_SendGoal_Response__Sequence__copy(
  const nav_targets__action__NavTarget_SendGoal_Response__Sequence * input,
  nav_targets__action__NavTarget_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(nav_targets__action__NavTarget_SendGoal_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    nav_targets__action__NavTarget_SendGoal_Response * data =
      (nav_targets__action__NavTarget_SendGoal_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!nav_targets__action__NavTarget_SendGoal_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          nav_targets__action__NavTarget_SendGoal_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!nav_targets__action__NavTarget_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "nav_targets/action/detail/nav_target__functions.h"

bool
nav_targets__action__NavTarget_SendGoal_Event__init(nav_targets__action__NavTarget_SendGoal_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    nav_targets__action__NavTarget_SendGoal_Event__fini(msg);
    return false;
  }
  // request
  if (!nav_targets__action__NavTarget_SendGoal_Request__Sequence__init(&msg->request, 0)) {
    nav_targets__action__NavTarget_SendGoal_Event__fini(msg);
    return false;
  }
  // response
  if (!nav_targets__action__NavTarget_SendGoal_Response__Sequence__init(&msg->response, 0)) {
    nav_targets__action__NavTarget_SendGoal_Event__fini(msg);
    return false;
  }
  return true;
}

void
nav_targets__action__NavTarget_SendGoal_Event__fini(nav_targets__action__NavTarget_SendGoal_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  nav_targets__action__NavTarget_SendGoal_Request__Sequence__fini(&msg->request);
  // response
  nav_targets__action__NavTarget_SendGoal_Response__Sequence__fini(&msg->response);
}

bool
nav_targets__action__NavTarget_SendGoal_Event__are_equal(const nav_targets__action__NavTarget_SendGoal_Event * lhs, const nav_targets__action__NavTarget_SendGoal_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!nav_targets__action__NavTarget_SendGoal_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!nav_targets__action__NavTarget_SendGoal_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
nav_targets__action__NavTarget_SendGoal_Event__copy(
  const nav_targets__action__NavTarget_SendGoal_Event * input,
  nav_targets__action__NavTarget_SendGoal_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!nav_targets__action__NavTarget_SendGoal_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!nav_targets__action__NavTarget_SendGoal_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

nav_targets__action__NavTarget_SendGoal_Event *
nav_targets__action__NavTarget_SendGoal_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nav_targets__action__NavTarget_SendGoal_Event * msg = (nav_targets__action__NavTarget_SendGoal_Event *)allocator.allocate(sizeof(nav_targets__action__NavTarget_SendGoal_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(nav_targets__action__NavTarget_SendGoal_Event));
  bool success = nav_targets__action__NavTarget_SendGoal_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
nav_targets__action__NavTarget_SendGoal_Event__destroy(nav_targets__action__NavTarget_SendGoal_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    nav_targets__action__NavTarget_SendGoal_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
nav_targets__action__NavTarget_SendGoal_Event__Sequence__init(nav_targets__action__NavTarget_SendGoal_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nav_targets__action__NavTarget_SendGoal_Event * data = NULL;

  if (size) {
    data = (nav_targets__action__NavTarget_SendGoal_Event *)allocator.zero_allocate(size, sizeof(nav_targets__action__NavTarget_SendGoal_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = nav_targets__action__NavTarget_SendGoal_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        nav_targets__action__NavTarget_SendGoal_Event__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
nav_targets__action__NavTarget_SendGoal_Event__Sequence__fini(nav_targets__action__NavTarget_SendGoal_Event__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      nav_targets__action__NavTarget_SendGoal_Event__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

nav_targets__action__NavTarget_SendGoal_Event__Sequence *
nav_targets__action__NavTarget_SendGoal_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nav_targets__action__NavTarget_SendGoal_Event__Sequence * array = (nav_targets__action__NavTarget_SendGoal_Event__Sequence *)allocator.allocate(sizeof(nav_targets__action__NavTarget_SendGoal_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = nav_targets__action__NavTarget_SendGoal_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
nav_targets__action__NavTarget_SendGoal_Event__Sequence__destroy(nav_targets__action__NavTarget_SendGoal_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    nav_targets__action__NavTarget_SendGoal_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
nav_targets__action__NavTarget_SendGoal_Event__Sequence__are_equal(const nav_targets__action__NavTarget_SendGoal_Event__Sequence * lhs, const nav_targets__action__NavTarget_SendGoal_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!nav_targets__action__NavTarget_SendGoal_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
nav_targets__action__NavTarget_SendGoal_Event__Sequence__copy(
  const nav_targets__action__NavTarget_SendGoal_Event__Sequence * input,
  nav_targets__action__NavTarget_SendGoal_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(nav_targets__action__NavTarget_SendGoal_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    nav_targets__action__NavTarget_SendGoal_Event * data =
      (nav_targets__action__NavTarget_SendGoal_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!nav_targets__action__NavTarget_SendGoal_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          nav_targets__action__NavTarget_SendGoal_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!nav_targets__action__NavTarget_SendGoal_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
nav_targets__action__NavTarget_GetResult_Request__init(nav_targets__action__NavTarget_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    nav_targets__action__NavTarget_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
nav_targets__action__NavTarget_GetResult_Request__fini(nav_targets__action__NavTarget_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
nav_targets__action__NavTarget_GetResult_Request__are_equal(const nav_targets__action__NavTarget_GetResult_Request * lhs, const nav_targets__action__NavTarget_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
nav_targets__action__NavTarget_GetResult_Request__copy(
  const nav_targets__action__NavTarget_GetResult_Request * input,
  nav_targets__action__NavTarget_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

nav_targets__action__NavTarget_GetResult_Request *
nav_targets__action__NavTarget_GetResult_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nav_targets__action__NavTarget_GetResult_Request * msg = (nav_targets__action__NavTarget_GetResult_Request *)allocator.allocate(sizeof(nav_targets__action__NavTarget_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(nav_targets__action__NavTarget_GetResult_Request));
  bool success = nav_targets__action__NavTarget_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
nav_targets__action__NavTarget_GetResult_Request__destroy(nav_targets__action__NavTarget_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    nav_targets__action__NavTarget_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
nav_targets__action__NavTarget_GetResult_Request__Sequence__init(nav_targets__action__NavTarget_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nav_targets__action__NavTarget_GetResult_Request * data = NULL;

  if (size) {
    data = (nav_targets__action__NavTarget_GetResult_Request *)allocator.zero_allocate(size, sizeof(nav_targets__action__NavTarget_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = nav_targets__action__NavTarget_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        nav_targets__action__NavTarget_GetResult_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
nav_targets__action__NavTarget_GetResult_Request__Sequence__fini(nav_targets__action__NavTarget_GetResult_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      nav_targets__action__NavTarget_GetResult_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

nav_targets__action__NavTarget_GetResult_Request__Sequence *
nav_targets__action__NavTarget_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nav_targets__action__NavTarget_GetResult_Request__Sequence * array = (nav_targets__action__NavTarget_GetResult_Request__Sequence *)allocator.allocate(sizeof(nav_targets__action__NavTarget_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = nav_targets__action__NavTarget_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
nav_targets__action__NavTarget_GetResult_Request__Sequence__destroy(nav_targets__action__NavTarget_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    nav_targets__action__NavTarget_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
nav_targets__action__NavTarget_GetResult_Request__Sequence__are_equal(const nav_targets__action__NavTarget_GetResult_Request__Sequence * lhs, const nav_targets__action__NavTarget_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!nav_targets__action__NavTarget_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
nav_targets__action__NavTarget_GetResult_Request__Sequence__copy(
  const nav_targets__action__NavTarget_GetResult_Request__Sequence * input,
  nav_targets__action__NavTarget_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(nav_targets__action__NavTarget_GetResult_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    nav_targets__action__NavTarget_GetResult_Request * data =
      (nav_targets__action__NavTarget_GetResult_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!nav_targets__action__NavTarget_GetResult_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          nav_targets__action__NavTarget_GetResult_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!nav_targets__action__NavTarget_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "nav_targets/action/detail/nav_target__functions.h"

bool
nav_targets__action__NavTarget_GetResult_Response__init(nav_targets__action__NavTarget_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!nav_targets__action__NavTarget_Result__init(&msg->result)) {
    nav_targets__action__NavTarget_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
nav_targets__action__NavTarget_GetResult_Response__fini(nav_targets__action__NavTarget_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  nav_targets__action__NavTarget_Result__fini(&msg->result);
}

bool
nav_targets__action__NavTarget_GetResult_Response__are_equal(const nav_targets__action__NavTarget_GetResult_Response * lhs, const nav_targets__action__NavTarget_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!nav_targets__action__NavTarget_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
nav_targets__action__NavTarget_GetResult_Response__copy(
  const nav_targets__action__NavTarget_GetResult_Response * input,
  nav_targets__action__NavTarget_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!nav_targets__action__NavTarget_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

nav_targets__action__NavTarget_GetResult_Response *
nav_targets__action__NavTarget_GetResult_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nav_targets__action__NavTarget_GetResult_Response * msg = (nav_targets__action__NavTarget_GetResult_Response *)allocator.allocate(sizeof(nav_targets__action__NavTarget_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(nav_targets__action__NavTarget_GetResult_Response));
  bool success = nav_targets__action__NavTarget_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
nav_targets__action__NavTarget_GetResult_Response__destroy(nav_targets__action__NavTarget_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    nav_targets__action__NavTarget_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
nav_targets__action__NavTarget_GetResult_Response__Sequence__init(nav_targets__action__NavTarget_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nav_targets__action__NavTarget_GetResult_Response * data = NULL;

  if (size) {
    data = (nav_targets__action__NavTarget_GetResult_Response *)allocator.zero_allocate(size, sizeof(nav_targets__action__NavTarget_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = nav_targets__action__NavTarget_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        nav_targets__action__NavTarget_GetResult_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
nav_targets__action__NavTarget_GetResult_Response__Sequence__fini(nav_targets__action__NavTarget_GetResult_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      nav_targets__action__NavTarget_GetResult_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

nav_targets__action__NavTarget_GetResult_Response__Sequence *
nav_targets__action__NavTarget_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nav_targets__action__NavTarget_GetResult_Response__Sequence * array = (nav_targets__action__NavTarget_GetResult_Response__Sequence *)allocator.allocate(sizeof(nav_targets__action__NavTarget_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = nav_targets__action__NavTarget_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
nav_targets__action__NavTarget_GetResult_Response__Sequence__destroy(nav_targets__action__NavTarget_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    nav_targets__action__NavTarget_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
nav_targets__action__NavTarget_GetResult_Response__Sequence__are_equal(const nav_targets__action__NavTarget_GetResult_Response__Sequence * lhs, const nav_targets__action__NavTarget_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!nav_targets__action__NavTarget_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
nav_targets__action__NavTarget_GetResult_Response__Sequence__copy(
  const nav_targets__action__NavTarget_GetResult_Response__Sequence * input,
  nav_targets__action__NavTarget_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(nav_targets__action__NavTarget_GetResult_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    nav_targets__action__NavTarget_GetResult_Response * data =
      (nav_targets__action__NavTarget_GetResult_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!nav_targets__action__NavTarget_GetResult_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          nav_targets__action__NavTarget_GetResult_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!nav_targets__action__NavTarget_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
// already included above
// #include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "nav_targets/action/detail/nav_target__functions.h"

bool
nav_targets__action__NavTarget_GetResult_Event__init(nav_targets__action__NavTarget_GetResult_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    nav_targets__action__NavTarget_GetResult_Event__fini(msg);
    return false;
  }
  // request
  if (!nav_targets__action__NavTarget_GetResult_Request__Sequence__init(&msg->request, 0)) {
    nav_targets__action__NavTarget_GetResult_Event__fini(msg);
    return false;
  }
  // response
  if (!nav_targets__action__NavTarget_GetResult_Response__Sequence__init(&msg->response, 0)) {
    nav_targets__action__NavTarget_GetResult_Event__fini(msg);
    return false;
  }
  return true;
}

void
nav_targets__action__NavTarget_GetResult_Event__fini(nav_targets__action__NavTarget_GetResult_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  nav_targets__action__NavTarget_GetResult_Request__Sequence__fini(&msg->request);
  // response
  nav_targets__action__NavTarget_GetResult_Response__Sequence__fini(&msg->response);
}

bool
nav_targets__action__NavTarget_GetResult_Event__are_equal(const nav_targets__action__NavTarget_GetResult_Event * lhs, const nav_targets__action__NavTarget_GetResult_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!nav_targets__action__NavTarget_GetResult_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!nav_targets__action__NavTarget_GetResult_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
nav_targets__action__NavTarget_GetResult_Event__copy(
  const nav_targets__action__NavTarget_GetResult_Event * input,
  nav_targets__action__NavTarget_GetResult_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!nav_targets__action__NavTarget_GetResult_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!nav_targets__action__NavTarget_GetResult_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

nav_targets__action__NavTarget_GetResult_Event *
nav_targets__action__NavTarget_GetResult_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nav_targets__action__NavTarget_GetResult_Event * msg = (nav_targets__action__NavTarget_GetResult_Event *)allocator.allocate(sizeof(nav_targets__action__NavTarget_GetResult_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(nav_targets__action__NavTarget_GetResult_Event));
  bool success = nav_targets__action__NavTarget_GetResult_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
nav_targets__action__NavTarget_GetResult_Event__destroy(nav_targets__action__NavTarget_GetResult_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    nav_targets__action__NavTarget_GetResult_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
nav_targets__action__NavTarget_GetResult_Event__Sequence__init(nav_targets__action__NavTarget_GetResult_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nav_targets__action__NavTarget_GetResult_Event * data = NULL;

  if (size) {
    data = (nav_targets__action__NavTarget_GetResult_Event *)allocator.zero_allocate(size, sizeof(nav_targets__action__NavTarget_GetResult_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = nav_targets__action__NavTarget_GetResult_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        nav_targets__action__NavTarget_GetResult_Event__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
nav_targets__action__NavTarget_GetResult_Event__Sequence__fini(nav_targets__action__NavTarget_GetResult_Event__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      nav_targets__action__NavTarget_GetResult_Event__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

nav_targets__action__NavTarget_GetResult_Event__Sequence *
nav_targets__action__NavTarget_GetResult_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nav_targets__action__NavTarget_GetResult_Event__Sequence * array = (nav_targets__action__NavTarget_GetResult_Event__Sequence *)allocator.allocate(sizeof(nav_targets__action__NavTarget_GetResult_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = nav_targets__action__NavTarget_GetResult_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
nav_targets__action__NavTarget_GetResult_Event__Sequence__destroy(nav_targets__action__NavTarget_GetResult_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    nav_targets__action__NavTarget_GetResult_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
nav_targets__action__NavTarget_GetResult_Event__Sequence__are_equal(const nav_targets__action__NavTarget_GetResult_Event__Sequence * lhs, const nav_targets__action__NavTarget_GetResult_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!nav_targets__action__NavTarget_GetResult_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
nav_targets__action__NavTarget_GetResult_Event__Sequence__copy(
  const nav_targets__action__NavTarget_GetResult_Event__Sequence * input,
  nav_targets__action__NavTarget_GetResult_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(nav_targets__action__NavTarget_GetResult_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    nav_targets__action__NavTarget_GetResult_Event * data =
      (nav_targets__action__NavTarget_GetResult_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!nav_targets__action__NavTarget_GetResult_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          nav_targets__action__NavTarget_GetResult_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!nav_targets__action__NavTarget_GetResult_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "nav_targets/action/detail/nav_target__functions.h"

bool
nav_targets__action__NavTarget_FeedbackMessage__init(nav_targets__action__NavTarget_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    nav_targets__action__NavTarget_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!nav_targets__action__NavTarget_Feedback__init(&msg->feedback)) {
    nav_targets__action__NavTarget_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
nav_targets__action__NavTarget_FeedbackMessage__fini(nav_targets__action__NavTarget_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  nav_targets__action__NavTarget_Feedback__fini(&msg->feedback);
}

bool
nav_targets__action__NavTarget_FeedbackMessage__are_equal(const nav_targets__action__NavTarget_FeedbackMessage * lhs, const nav_targets__action__NavTarget_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!nav_targets__action__NavTarget_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
nav_targets__action__NavTarget_FeedbackMessage__copy(
  const nav_targets__action__NavTarget_FeedbackMessage * input,
  nav_targets__action__NavTarget_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!nav_targets__action__NavTarget_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

nav_targets__action__NavTarget_FeedbackMessage *
nav_targets__action__NavTarget_FeedbackMessage__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nav_targets__action__NavTarget_FeedbackMessage * msg = (nav_targets__action__NavTarget_FeedbackMessage *)allocator.allocate(sizeof(nav_targets__action__NavTarget_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(nav_targets__action__NavTarget_FeedbackMessage));
  bool success = nav_targets__action__NavTarget_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
nav_targets__action__NavTarget_FeedbackMessage__destroy(nav_targets__action__NavTarget_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    nav_targets__action__NavTarget_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
nav_targets__action__NavTarget_FeedbackMessage__Sequence__init(nav_targets__action__NavTarget_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nav_targets__action__NavTarget_FeedbackMessage * data = NULL;

  if (size) {
    data = (nav_targets__action__NavTarget_FeedbackMessage *)allocator.zero_allocate(size, sizeof(nav_targets__action__NavTarget_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = nav_targets__action__NavTarget_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        nav_targets__action__NavTarget_FeedbackMessage__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
nav_targets__action__NavTarget_FeedbackMessage__Sequence__fini(nav_targets__action__NavTarget_FeedbackMessage__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      nav_targets__action__NavTarget_FeedbackMessage__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

nav_targets__action__NavTarget_FeedbackMessage__Sequence *
nav_targets__action__NavTarget_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nav_targets__action__NavTarget_FeedbackMessage__Sequence * array = (nav_targets__action__NavTarget_FeedbackMessage__Sequence *)allocator.allocate(sizeof(nav_targets__action__NavTarget_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = nav_targets__action__NavTarget_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
nav_targets__action__NavTarget_FeedbackMessage__Sequence__destroy(nav_targets__action__NavTarget_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    nav_targets__action__NavTarget_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
nav_targets__action__NavTarget_FeedbackMessage__Sequence__are_equal(const nav_targets__action__NavTarget_FeedbackMessage__Sequence * lhs, const nav_targets__action__NavTarget_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!nav_targets__action__NavTarget_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
nav_targets__action__NavTarget_FeedbackMessage__Sequence__copy(
  const nav_targets__action__NavTarget_FeedbackMessage__Sequence * input,
  nav_targets__action__NavTarget_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(nav_targets__action__NavTarget_FeedbackMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    nav_targets__action__NavTarget_FeedbackMessage * data =
      (nav_targets__action__NavTarget_FeedbackMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!nav_targets__action__NavTarget_FeedbackMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          nav_targets__action__NavTarget_FeedbackMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!nav_targets__action__NavTarget_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
