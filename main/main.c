#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "bdc_motor.h"
#include "pid_ctrl.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/joint_state.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "rosidl_runtime_c/string_functions.h"

static const char *TAG = "Amanita Robot";

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn)                                                                      \
    {                                                                                    \
        rcl_ret_t temp_rc = fn;                                                          \
        if ((temp_rc != RCL_RET_OK))                                                     \
        {                                                                                \
            printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
            vTaskDelete(NULL);                                                           \
        }                                                                                \
    }

#define RCSOFTCHECK(fn)                                                                    \
    {                                                                                      \
        rcl_ret_t temp_rc = fn;                                                            \
        if ((temp_rc != RCL_RET_OK))                                                       \
        {                                                                                  \
            printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
        }                                                                                  \
    }

// PWM definitions
#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ 25000                // 25KHz PWM

// maximum duty cycle value in ticks
#define BDC_MCPWM_DUTY_TICK_MAX ((BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) - 1)

// Encoder definitions
#define BDC_ENCODER_PCNT_HIGH_LIMIT CONFIG_ENCODER_CPR
#define BDC_ENCODER_PCNT_LOW_LIMIT -CONFIG_ENCODER_CPR

// PID loop definitions
#define BDC_PID_LOOP_PERIOD_MS 10 // calculate the motor speed every 10ms

// Velocity conversion macros. MPS = m/s. COUNTPP = counts per PID loop period.
#define DISTANCE(countpp) (2.0 * M_PI * CONFIG_WHEEL_RADIUS * countpp) / (CONFIG_ENCODER_CPR * 1000000)
#define COUNTPP_TO_MPS(countpp) (2.0 * M_PI * CONFIG_WHEEL_RADIUS * countpp) / (CONFIG_ENCODER_CPR * BDC_PID_LOOP_PERIOD_MS * 1000)
#define MPS_TO_COUNTPP(mps) (mps * CONFIG_ENCODER_CPR * BDC_PID_LOOP_PERIOD_MS * 1000.0) / (2.0 * M_PI * CONFIG_WHEEL_RADIUS)
#define TOGGLE(n) n ? 0 : 1

#ifdef CONFIG_MOTOR_LEFT_INVERTED
int left_motor_pin_a = CONFIG_MOTOR_LEFT_MCPWM_B;
int left_motor_pin_b = CONFIG_MOTOR_LEFT_MCPWM_A;
int left_encoder_pin_a = CONFIG_ENCODER_LEFT_CHANNEL_B;
int left_encoder_pin_b = CONFIG_ENCODER_LEFT_CHANNEL_A;
#else
int left_motor_pin_a = CONFIG_MOTOR_LEFT_MCPWM_A;
int left_motor_pin_b = CONFIG_MOTOR_LEFT_MCPWM_B;
int left_encoder_pin_a = CONFIG_ENCODER_LEFT_CHANNEL_A;
int left_encoder_pin_b = CONFIG_ENCODER_LEFT_CHANNEL_B;
#endif // CONFIG_MOTOR_LEFT_INVERTED

#ifdef CONFIG_MOTOR_RIGHT_INVERTED
int right_motor_pin_a = CONFIG_MOTOR_RIGHT_MCPWM_B;
int right_motor_pin_b = CONFIG_MOTOR_RIGHT_MCPWM_A;
int right_encoder_pin_a = CONFIG_ENCODER_RIGHT_CHANNEL_B;
int right_encoder_pin_b = CONFIG_ENCODER_RIGHT_CHANNEL_A;
#else
int right_motor_pin_a = CONFIG_MOTOR_RIGHT_MCPWM_A;
int right_motor_pin_b = CONFIG_MOTOR_RIGHT_MCPWM_B;
int right_encoder_pin_a = CONFIG_ENCODER_RIGHT_CHANNEL_A;
int right_encoder_pin_b = CONFIG_ENCODER_RIGHT_CHANNEL_B;
#endif // CONFIG_MOTOR_RIGHT_INVERTED

static int level_one = 0;
static int level_two = 0;

enum axis
{
    LEFT = 0,
    RIGHT = 1
};

typedef struct
{
    bdc_motor_handle_t left_motor;
    pcnt_unit_handle_t left_pcnt_encoder;
    pid_ctrl_block_handle_t left_pid_ctrl;
    int left_reported_pulses;
    bdc_motor_handle_t right_motor;
    pcnt_unit_handle_t right_pcnt_encoder;
    pid_ctrl_block_handle_t right_pid_ctrl;
    int right_reported_pulses;
} robot_control_context_t;

rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist twist_msg;
static double vel_setpoint[2] = {0.0, 0.0};

rcl_publisher_t joint_state_publisher;
sensor_msgs__msg__JointState joint_state_msg;
static double pos[2] = {0.0, 0.0};
static double vel[2] = {0.0, 0.0};
static double pwms[2] = {0.0, 0.0};
const char *names[2] = {"L_axis", "R_axis"};
const char *frame_id = "/amanita";
bool torque_state = false;

bool set_torque(bool state)
{
    gpio_set_level(CONFIG_MOTOR_LEFT_ENABLE, state);
    gpio_set_level(CONFIG_MOTOR_RIGHT_ENABLE, state);
    ESP_LOGI(TAG, "Motors %s.", state ? "enabled" : "disabled");
    return state;
}

static void pid_loop_cb(void *args)
{
    robot_control_context_t *ctx = (robot_control_context_t *)args;

    static int left_last_pulse_count = 0;
    pcnt_unit_handle_t left_pcnt_unit = ctx->left_pcnt_encoder;
    pid_ctrl_block_handle_t left_pid_ctrl = ctx->left_pid_ctrl;
    bdc_motor_handle_t left_motor = ctx->left_motor;

    static int right_last_pulse_count = 0;
    pcnt_unit_handle_t right_pcnt_unit = ctx->right_pcnt_encoder;
    pid_ctrl_block_handle_t right_pid_ctrl = ctx->right_pid_ctrl;
    bdc_motor_handle_t right_motor = ctx->right_motor;

    // Get the result from rotary encoders.
    int cur_pulse_count = 0;
    pcnt_unit_get_count(left_pcnt_unit, &cur_pulse_count);
    int left_real_pulses = cur_pulse_count - left_last_pulse_count;
    left_last_pulse_count = cur_pulse_count;
    ctx->left_reported_pulses = left_real_pulses;

    cur_pulse_count = 0;
    pcnt_unit_get_count(right_pcnt_unit, &cur_pulse_count);
    int right_real_pulses = cur_pulse_count - right_last_pulse_count;
    right_last_pulse_count = cur_pulse_count;
    ctx->right_reported_pulses = right_real_pulses;

    // Convert Wheel velocity from counts/period_ms to m/s.
    vel[LEFT] = COUNTPP_TO_MPS(left_real_pulses);
    vel[RIGHT] = COUNTPP_TO_MPS(right_real_pulses);

    // Calculate the speed error.
    float left_error = MPS_TO_COUNTPP(vel_setpoint[LEFT]) - left_real_pulses;
    float right_error = MPS_TO_COUNTPP(vel_setpoint[RIGHT]) - right_real_pulses;
    float left_new_vel = 0;
    float right_new_vel = 0;

    // Compute PIDs.
    if (torque_state == true)
    {
        pid_compute(left_pid_ctrl, left_error, &left_new_vel);
        pid_compute(right_pid_ctrl, right_error, &right_new_vel);
    }

    // Store PWM and distance values
    pwms[LEFT] = left_new_vel;
    pwms[RIGHT] = right_new_vel;
    pos[LEFT] += DISTANCE(left_real_pulses);
    pos[RIGHT] += DISTANCE(right_real_pulses);

    // Set motor directions.
    if (left_new_vel < 0)
    {
        ESP_ERROR_CHECK(bdc_motor_reverse(left_motor));
    }
    else
    {
        ESP_ERROR_CHECK(bdc_motor_forward(left_motor));
    }

    if (right_new_vel < 0)
    {
        ESP_ERROR_CHECK(bdc_motor_reverse(right_motor));
    }
    else
    {
        ESP_ERROR_CHECK(bdc_motor_forward(right_motor));
    }

    // Set the new speeds
    bdc_motor_set_speed(left_motor, (uint32_t)fabs(left_new_vel));
    bdc_motor_set_speed(right_motor, (uint32_t)fabs(right_new_vel));

    level_two = TOGGLE(level_two);
    gpio_set_level(15, TOGGLE(level_two));
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    joint_state_msg.header.stamp.sec = ts.tv_sec;
    joint_state_msg.header.stamp.nanosec = ts.tv_nsec;
    RCSOFTCHECK(rcl_publish(&joint_state_publisher, &joint_state_msg, NULL));
    level_one = TOGGLE(level_one);
    gpio_set_level(14, level_one);
}

void cmd_vel_subscrition_cb(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    bool disabled = false;
    if (msg->linear.y == 99.99 && msg->angular.y == 99.99)
    {
        torque_state = set_torque(false);
        disabled = true;
    }
    if (msg->linear.y == 77.77 && msg->angular.y == 77.77)
    {
        torque_state = set_torque(true);
    }
    if (torque_state == true)
    {
        vel_setpoint[LEFT] = (msg->linear.x - (msg->angular.z * ((float)CONFIG_WHEELS_DISTANCE / 1000000) / 2.0));
        vel_setpoint[RIGHT] = (msg->linear.x + (msg->angular.z * ((float)CONFIG_WHEELS_DISTANCE / 1000000) / 2.0));
        ESP_LOGI(TAG, "cmd_vel.linear: %f, cmd_vel.angular: %f", msg->linear.x, msg->linear.z);
        ESP_LOGI(TAG, "vel_setpoint[LEFT]: %f, vel_setpoint[RIGHT]: %f", vel_setpoint[LEFT], vel_setpoint[RIGHT]);
    }
    if (torque_state == false && disabled == false)
    {
        ESP_LOGI(TAG, "Motors disabled. Command not accepted.");
    }
}

void micro_ros_task(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // Create init_options.
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

    // Static Agent IP and port can be used instead of autodisvery.
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
    // RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif

    // Setup support structure.
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // Create node.
    rcl_node_t node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&node, "amanita_uros_espidf_rclc", "", &support));

    // Create publisher.
    RCCHECK(rclc_publisher_init_default(
        &joint_state_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "/amanita/joint_states"));

    // Create subscriber.
    RCCHECK(rclc_subscription_init_best_effort(
        &cmd_vel_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/amanita/cmd_vel"));

    // Create timer.
    rcl_timer_t timer = rcl_get_zero_initialized_timer();
    const unsigned int timer_timeout = 1000;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));

    // Create executor.
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    unsigned int rcl_wait_timeout = 1000; // in ms
    RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

    // Add timer and subscriber to executor.
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber,
                                           &twist_msg, &cmd_vel_subscrition_cb, ON_NEW_DATA));

    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(100000);
    }

    // Free resources.
    RCCHECK(rcl_subscription_fini(&cmd_vel_subscriber, &node));
    RCCHECK(rcl_publisher_fini(&joint_state_publisher, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}

void app_main(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    // GPIO Initialization
    gpio_reset_pin(CONFIG_MOTOR_LEFT_ENABLE);
    gpio_set_direction(CONFIG_MOTOR_LEFT_ENABLE, GPIO_MODE_OUTPUT);
    gpio_reset_pin(CONFIG_MOTOR_RIGHT_ENABLE);
    gpio_set_direction(CONFIG_MOTOR_RIGHT_ENABLE, GPIO_MODE_OUTPUT);

    gpio_reset_pin(14);
    gpio_set_direction(14, GPIO_MODE_OUTPUT);

    gpio_reset_pin(15);
    gpio_set_direction(15, GPIO_MODE_OUTPUT);

    static robot_control_context_t robot_ctrl_ctx = {
        .left_pcnt_encoder = NULL,
        .right_pcnt_encoder = NULL,
    };

    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };

    ESP_LOGI(TAG, "Create Left DC motor");
    bdc_motor_config_t left_motor_config = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = left_motor_pin_a,
        .pwmb_gpio_num = left_motor_pin_b,
    };

    bdc_motor_handle_t left_motor = NULL;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&left_motor_config, &mcpwm_config, &left_motor));
    robot_ctrl_ctx.left_motor = left_motor;

    ESP_LOGI(TAG, "Create Right DC motor");
    bdc_motor_config_t right_motor_config = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = right_motor_pin_a,
        .pwmb_gpio_num = right_motor_pin_b,
    };

    bdc_motor_handle_t right_motor = NULL;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&right_motor_config, &mcpwm_config, &right_motor));
    robot_ctrl_ctx.right_motor = right_motor;

    pcnt_unit_config_t unit_config = {
        .high_limit = BDC_ENCODER_PCNT_HIGH_LIMIT,
        .low_limit = BDC_ENCODER_PCNT_LOW_LIMIT,
        .flags.accum_count = true, // enable counter accumulation
    };

    ESP_LOGI(TAG, "Init Left pcnt driver to decode rotary signal");
    pcnt_unit_handle_t left_pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &left_pcnt_unit));
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(left_pcnt_unit, &filter_config));
    pcnt_chan_config_t left_chan_a_config = {
        .edge_gpio_num = left_encoder_pin_a,
        .level_gpio_num = left_encoder_pin_b,
    };
    pcnt_channel_handle_t left_pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(left_pcnt_unit, &left_chan_a_config, &left_pcnt_chan_a));
    pcnt_chan_config_t left_chan_b_config = {
        .edge_gpio_num = left_encoder_pin_b,
        .level_gpio_num = left_encoder_pin_a,
    };
    pcnt_channel_handle_t left_pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(left_pcnt_unit, &left_chan_b_config, &left_pcnt_chan_b));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(left_pcnt_chan_a,
                                                 PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(left_pcnt_chan_a,
                                                  PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(left_pcnt_chan_b,
                                                 PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(left_pcnt_chan_b,
                                                  PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(left_pcnt_unit, BDC_ENCODER_PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(left_pcnt_unit, BDC_ENCODER_PCNT_LOW_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_enable(left_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(left_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(left_pcnt_unit));
    robot_ctrl_ctx.left_pcnt_encoder = left_pcnt_unit;

    ESP_LOGI(TAG, "Init Right pcnt driver to decode rotary signal");
    pcnt_unit_handle_t right_pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &right_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(right_pcnt_unit, &filter_config));
    pcnt_chan_config_t right_chan_a_config = {
        .edge_gpio_num = right_encoder_pin_a,
        .level_gpio_num = right_encoder_pin_b,
    };
    pcnt_channel_handle_t right_pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(right_pcnt_unit, &right_chan_a_config, &right_pcnt_chan_a));
    pcnt_chan_config_t right_chan_b_config = {
        .edge_gpio_num = right_encoder_pin_b,
        .level_gpio_num = right_encoder_pin_a,
    };
    pcnt_channel_handle_t right_pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(right_pcnt_unit, &right_chan_b_config, &right_pcnt_chan_b));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(right_pcnt_chan_a,
                                                 PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(right_pcnt_chan_a,
                                                  PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(right_pcnt_chan_b,
                                                 PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(right_pcnt_chan_b,
                                                  PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(right_pcnt_unit, BDC_ENCODER_PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(right_pcnt_unit, BDC_ENCODER_PCNT_LOW_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_enable(right_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(right_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(right_pcnt_unit));
    robot_ctrl_ctx.right_pcnt_encoder = right_pcnt_unit;

    ESP_LOGI(TAG, "Create Left PID control block");
    pid_ctrl_parameter_t left_pid_runtime_param = {
        .kp = (float)CONFIG_MOTOR_LEFT_P_GAIN / 1000.0,
        .ki = (float)CONFIG_MOTOR_LEFT_I_GAIN / 1000.0,
        .kd = (float)CONFIG_MOTOR_LEFT_D_GAIN / 1000.0,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .max_output = BDC_MCPWM_DUTY_TICK_MAX,
        .min_output = -BDC_MCPWM_DUTY_TICK_MAX,
        .max_integral = 1000,
        .min_integral = -1000,
    };
    pid_ctrl_block_handle_t left_pid_ctrl = NULL;
    pid_ctrl_config_t left_pid_config = {
        .init_param = left_pid_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&left_pid_config, &left_pid_ctrl));
    robot_ctrl_ctx.left_pid_ctrl = left_pid_ctrl;

    ESP_LOGI(TAG, "Create Right PID control block");
    pid_ctrl_parameter_t right_pid_runtime_param = {
        .kp = (float)CONFIG_MOTOR_RIGHT_P_GAIN / 1000.0,
        .ki = (float)CONFIG_MOTOR_RIGHT_I_GAIN / 1000.0,
        .kd = (float)CONFIG_MOTOR_RIGHT_D_GAIN / 1000.0,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .max_output = BDC_MCPWM_DUTY_TICK_MAX,
        .min_output = -BDC_MCPWM_DUTY_TICK_MAX,
        .max_integral = 1000,
        .min_integral = -1000,
    };
    pid_ctrl_block_handle_t right_pid_ctrl = NULL;
    pid_ctrl_config_t right_pid_config = {
        .init_param = right_pid_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&right_pid_config, &right_pid_ctrl));
    robot_ctrl_ctx.right_pid_ctrl = right_pid_ctrl;

    ESP_LOGI(TAG, "Create a timer to do PID calculation periodically");
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = pid_loop_cb,
        .arg = &robot_ctrl_ctx,
        .name = "pid_loop"};
    esp_timer_handle_t pid_loop_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &pid_loop_timer));

    ESP_LOGI(TAG, "Enable motors");
    ESP_ERROR_CHECK(bdc_motor_enable(left_motor));
    ESP_ERROR_CHECK(bdc_motor_enable(right_motor));
    torque_state = set_torque(true);

    // Initial Joint State message setup.
    rosidl_runtime_c__String__assign(&joint_state_msg.header.frame_id, frame_id);
    sensor_msgs__msg__JointState__init(&joint_state_msg);
    joint_state_msg.position.data = pos;
    joint_state_msg.position.size = 2;
    joint_state_msg.position.capacity = 2;
    joint_state_msg.velocity.data = vel;
    joint_state_msg.velocity.size = 2;
    joint_state_msg.velocity.capacity = 2;
    joint_state_msg.effort.data = pwms; // Sending PWM values instead
    joint_state_msg.effort.size = 2;
    joint_state_msg.effort.capacity = 2;
    rosidl_runtime_c__String__Sequence__init(&joint_state_msg.name, 2);
    rosidl_runtime_c__String__assign(&joint_state_msg.name.data[LEFT], names[LEFT]);
    rosidl_runtime_c__String__assign(&joint_state_msg.name.data[RIGHT], names[RIGHT]);

    ESP_LOGI(TAG, "Starting motor control loop.");
    ESP_ERROR_CHECK(esp_timer_start_periodic(pid_loop_timer, BDC_PID_LOOP_PERIOD_MS * 1000));

    // pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(micro_ros_task,
                "uros_task",
                CONFIG_MICRO_ROS_APP_STACK,
                NULL,
                CONFIG_MICRO_ROS_APP_TASK_PRIO,
                NULL);

    gpio_reset_pin(CONFIG_LED_ONBOARD);
    gpio_set_direction(CONFIG_LED_ONBOARD, GPIO_MODE_OUTPUT);
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level(CONFIG_LED_ONBOARD, 1);
        vTaskDelay(pdMS_TO_TICKS(950));
        gpio_set_level(CONFIG_LED_ONBOARD, 0);
    }
}