// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from nav_targets:action/NavTarget.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "nav_targets/action/nav_target.h"


#ifndef NAV_TARGETS__ACTION__DETAIL__NAV_TARGET__STRUCT_H_
#define NAV_TARGETS__ACTION__DETAIL__NAV_TARGET__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'goal'
#include "geometry_msgs/msg/detail/point_stamped__struct.h"

/// Struct defined in action/NavTarget in the package nav_targets.
typedef struct nav_targets__action__NavTarget_Goal
{
  geometry_msgs__msg__PointStamped goal;
} nav_targets__action__NavTarget_Goal;

// Struct for a sequence of nav_targets__action__NavTarget_Goal.
typedef struct nav_targets__action__NavTarget_Goal__Sequence
{
  nav_targets__action__NavTarget_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nav_targets__action__NavTarget_Goal__Sequence;

// Constants defined in the message

/// Struct defined in action/NavTarget in the package nav_targets.
typedef struct nav_targets__action__NavTarget_Result
{
  bool success;
} nav_targets__action__NavTarget_Result;

// Struct for a sequence of nav_targets__action__NavTarget_Result.
typedef struct nav_targets__action__NavTarget_Result__Sequence
{
  nav_targets__action__NavTarget_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nav_targets__action__NavTarget_Result__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'distance'
#include "std_msgs/msg/detail/float32__struct.h"

/// Struct defined in action/NavTarget in the package nav_targets.
typedef struct nav_targets__action__NavTarget_Feedback
{
  std_msgs__msg__Float32 distance;
} nav_targets__action__NavTarget_Feedback;

// Struct for a sequence of nav_targets__action__NavTarget_Feedback.
typedef struct nav_targets__action__NavTarget_Feedback__Sequence
{
  nav_targets__action__NavTarget_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nav_targets__action__NavTarget_Feedback__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "nav_targets/action/detail/nav_target__struct.h"

/// Struct defined in action/NavTarget in the package nav_targets.
typedef struct nav_targets__action__NavTarget_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  nav_targets__action__NavTarget_Goal goal;
} nav_targets__action__NavTarget_SendGoal_Request;

// Struct for a sequence of nav_targets__action__NavTarget_SendGoal_Request.
typedef struct nav_targets__action__NavTarget_SendGoal_Request__Sequence
{
  nav_targets__action__NavTarget_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nav_targets__action__NavTarget_SendGoal_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/NavTarget in the package nav_targets.
typedef struct nav_targets__action__NavTarget_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} nav_targets__action__NavTarget_SendGoal_Response;

// Struct for a sequence of nav_targets__action__NavTarget_SendGoal_Response.
typedef struct nav_targets__action__NavTarget_SendGoal_Response__Sequence
{
  nav_targets__action__NavTarget_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nav_targets__action__NavTarget_SendGoal_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  nav_targets__action__NavTarget_SendGoal_Event__request__MAX_SIZE = 1
};
// response
enum
{
  nav_targets__action__NavTarget_SendGoal_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/NavTarget in the package nav_targets.
typedef struct nav_targets__action__NavTarget_SendGoal_Event
{
  service_msgs__msg__ServiceEventInfo info;
  nav_targets__action__NavTarget_SendGoal_Request__Sequence request;
  nav_targets__action__NavTarget_SendGoal_Response__Sequence response;
} nav_targets__action__NavTarget_SendGoal_Event;

// Struct for a sequence of nav_targets__action__NavTarget_SendGoal_Event.
typedef struct nav_targets__action__NavTarget_SendGoal_Event__Sequence
{
  nav_targets__action__NavTarget_SendGoal_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nav_targets__action__NavTarget_SendGoal_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/NavTarget in the package nav_targets.
typedef struct nav_targets__action__NavTarget_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} nav_targets__action__NavTarget_GetResult_Request;

// Struct for a sequence of nav_targets__action__NavTarget_GetResult_Request.
typedef struct nav_targets__action__NavTarget_GetResult_Request__Sequence
{
  nav_targets__action__NavTarget_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nav_targets__action__NavTarget_GetResult_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "nav_targets/action/detail/nav_target__struct.h"

/// Struct defined in action/NavTarget in the package nav_targets.
typedef struct nav_targets__action__NavTarget_GetResult_Response
{
  int8_t status;
  nav_targets__action__NavTarget_Result result;
} nav_targets__action__NavTarget_GetResult_Response;

// Struct for a sequence of nav_targets__action__NavTarget_GetResult_Response.
typedef struct nav_targets__action__NavTarget_GetResult_Response__Sequence
{
  nav_targets__action__NavTarget_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nav_targets__action__NavTarget_GetResult_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
// already included above
// #include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  nav_targets__action__NavTarget_GetResult_Event__request__MAX_SIZE = 1
};
// response
enum
{
  nav_targets__action__NavTarget_GetResult_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/NavTarget in the package nav_targets.
typedef struct nav_targets__action__NavTarget_GetResult_Event
{
  service_msgs__msg__ServiceEventInfo info;
  nav_targets__action__NavTarget_GetResult_Request__Sequence request;
  nav_targets__action__NavTarget_GetResult_Response__Sequence response;
} nav_targets__action__NavTarget_GetResult_Event;

// Struct for a sequence of nav_targets__action__NavTarget_GetResult_Event.
typedef struct nav_targets__action__NavTarget_GetResult_Event__Sequence
{
  nav_targets__action__NavTarget_GetResult_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nav_targets__action__NavTarget_GetResult_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "nav_targets/action/detail/nav_target__struct.h"

/// Struct defined in action/NavTarget in the package nav_targets.
typedef struct nav_targets__action__NavTarget_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  nav_targets__action__NavTarget_Feedback feedback;
} nav_targets__action__NavTarget_FeedbackMessage;

// Struct for a sequence of nav_targets__action__NavTarget_FeedbackMessage.
typedef struct nav_targets__action__NavTarget_FeedbackMessage__Sequence
{
  nav_targets__action__NavTarget_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nav_targets__action__NavTarget_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // NAV_TARGETS__ACTION__DETAIL__NAV_TARGET__STRUCT_H_
