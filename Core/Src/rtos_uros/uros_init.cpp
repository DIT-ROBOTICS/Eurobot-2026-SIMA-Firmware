/*
 * uros_init.cpp
 *
 *  Created on: Apr 9, 2025
 *      Author: stanly
 */


#include "uros_init.h"
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <string.h>

rcl_publisher_t                  mission_status_pub;
std_msgs__msg__Int32             mission_status_msg;
rcl_publisher_t                  odom_msg_pub;
nav_msgs__msg__Odometry          odom_msg;
rcl_publisher_t                  vl53l0x_pub;
std_msgs__msg__Float32MultiArray vl53l0x_msg;
rcl_subscription_t               cmd_vel_sub;
geometry_msgs__msg__Twist        cmd_vel_msg;
rcl_subscription_t               mission_type_sub;
std_msgs__msg__Int32             mission_type_msg;
rcl_timer_t                      uros_timer;

extern float V_Linear_goal;
extern float W_angular_goal;
extern float V_Linear_now;
extern float W_angular_now;
extern float vl53l0x_ranges[3];
rcl_ret_t pub_success = RCL_RET_OK;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_init_options_t init_options;
rclc_executor_t executor;

agent_status_t status = AGENT_WAITING;

int ping_fail_count = 0;
#define MAX_PING_FAIL_COUNT 5


extern UART_HandleTypeDef USARTx;

void uros_init(void) {
  // Initialize micro-ROS
  rmw_uros_set_custom_transport(
    true,
    (void *) &USARTx,
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read);
  
  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();

  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
  printf("Error on default allocators (line %d)\n", __LINE__); 
  }
}

void uros_agent_status_check(void) {
  switch (status) {
    case AGENT_WAITING:
      handle_state_agent_waiting();
      break;
    case AGENT_AVAILABLE:
      handle_state_agent_available();
      break;
    case AGENT_CONNECTED:
      handle_state_agent_connected();
      break;
    case AGENT_TRYING:
      handle_state_agent_trying();
      break;
    case AGENT_DISCONNECTED:
      handle_state_agent_disconnected();
      break;
    default:
      break;
  }
}

void handle_state_agent_waiting(void) {
  status = (rmw_uros_ping_agent(10, 10) == RMW_RET_OK) ? AGENT_AVAILABLE : AGENT_WAITING;
}

void handle_state_agent_available(void) {
  uros_create_entities();
  status = AGENT_CONNECTED;
}

void handle_state_agent_connected(void) {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}

void handle_state_agent_trying(void) {
  if(rmw_uros_ping_agent(1, 10) == RMW_RET_OK){
    status = AGENT_CONNECTED;
    ping_fail_count = 0; // Reset ping fail count
  } else {
    ping_fail_count++;
    if(ping_fail_count >= MAX_PING_FAIL_COUNT){
      status = AGENT_DISCONNECTED;
      ping_fail_count = 0;
    }
  }
}

void handle_state_agent_disconnected(void) {
  uros_destroy_entities();
  status = AGENT_WAITING;
}

void uros_create_entities(void) {
  allocator = rcl_get_default_allocator();

  init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, DOMAIN_ID);

  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator); // Initialize support structure

  rcl_init_options_fini(&init_options);
  
  rclc_node_init_default(&node, NODE_NAME, "", &support); // Initialize node
    
  rclc_publisher_init_default(
    &mission_status_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "mission_status");
    mission_status_msg.data = 0;

  rclc_publisher_init_default( // Initialize publisher for start message
    &odom_msg_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "wheel/odom");
  odom_msg.header.frame_id.data = "odom";
  odom_msg.child_frame_id.data = "base_link";
  odom_msg.pose.pose.position.x = 0.0;
  odom_msg.pose.pose.position.y = 0.0;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = 0.0;
  odom_msg.pose.pose.orientation.w = 1.0;
  odom_msg.twist.twist.linear.x = 0.0;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.linear.z = 0.0;
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z = 0.0;
  memset(odom_msg.pose.covariance, 0, sizeof(odom_msg.pose.covariance));
  memset(odom_msg.twist.covariance, 0, sizeof(odom_msg.twist.covariance));
  odom_msg.twist.covariance[0] = 0.02 * 0.02;   // vx variance
  odom_msg.twist.covariance[7] = 0.02 * 0.02;   // vy variance
  odom_msg.twist.covariance[14] = 1e-6;         // vz variance (small but non-zero for 2D)
  odom_msg.twist.covariance[21] = 1e-6;         // roll rate variance (small but non-zero for 2D)
  odom_msg.twist.covariance[28] = 1e-6;         // pitch rate variance (small but non-zero for 2D)
  odom_msg.twist.covariance[35] = 0.02 * 0.02;  // yaw rate variance

  rclc_publisher_init_default(
    &vl53l0x_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "sensors/raw_ranges");
  vl53l0x_msg.data.data = (float *)malloc(3 * sizeof(float));
  vl53l0x_msg.data.size = 3;
  vl53l0x_msg.data.capacity = 3;



  rclc_subscription_init_default( // Initialize subscriber for mission type
    &cmd_vel_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel");
  cmd_vel_msg.linear.x = 0.0;
  cmd_vel_msg.linear.y = 0.0;
  cmd_vel_msg.linear.z = 0.0;
  cmd_vel_msg.angular.x = 0.0;
  cmd_vel_msg.angular.y = 0.0;
  cmd_vel_msg.angular.z = 0.0;

  rclc_subscription_init_default( // Initialize subscriber for mission type
    &mission_type_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "robot/startup/sima_game_over");
  mission_type_msg.data = 0;

  rclc_timer_init_default(&uros_timer, &support, RCL_MS_TO_NS(1000/FREQUENCY), uros_timer_callback); // Initialize uros timer

  rclc_executor_init(&executor, &support.context, 3, &allocator); // Create executor

  rclc_executor_add_timer(&executor, &uros_timer);
  rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &mission_type_sub, &mission_type_msg, &mission_type_callback, ON_NEW_DATA);
}

void uros_destroy_entities(void) {
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  // Destroy timer
  rcl_timer_fini(&uros_timer);

  // Destroy publisher
  rcl_publisher_fini(&mission_status_pub, &node);
  rcl_publisher_fini(&odom_msg_pub, &node);

  // Destroy subscriber
  rcl_subscription_fini(&cmd_vel_sub, &node);
  rcl_subscription_fini(&mission_type_sub, &node);

  // Destroy executor
  rclc_executor_fini(&executor);

  // Destroy node
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}


void uros_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  // This function can be used for periodic tasks if needed
  odom_msg.twist.twist.linear.x = V_Linear_now;
  odom_msg.twist.twist.angular.z = W_angular_now;
  pub_success = rcl_publish(&odom_msg_pub, &odom_msg, NULL);
  if(pub_success != RCL_RET_OK){
    status = AGENT_TRYING;
  }
  for(int i = 0; i < 3; i++){
    vl53l0x_msg.data.data[i] = vl53l0x_ranges[i];
  }
  pub_success = rcl_publish(&vl53l0x_pub, &vl53l0x_msg, NULL);
  if(pub_success != RCL_RET_OK){
    status = AGENT_TRYING;
  }
  
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, mission_type_msg.data == 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);

  // status = (rmw_uros_ping_agent(10, 10) == RMW_RET_OK) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
  // mission status need to be added
}

void cmd_vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  V_Linear_goal = msg->linear.x;
  W_angular_goal = msg->angular.z;
}

void mission_type_callback(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  mission_type_msg.data = msg->data;
}
