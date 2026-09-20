//
// Created by elsa on 25-1-31.
//

#include "serial_pro/robot_serial.h"

void RobotSerial::velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    static uint8_t SOF = 0x00;
    Velocity velocity{
        (float)(msg->linear.x),
        (float)(msg->linear.y),
        (float)(msg->angular.z),
    };
    SOF++;
    sentrySerial.write(0x0501, SOF, velocity);
    RCLCPP_INFO(this->get_logger(), "%f %f %f", msg->linear.x, msg->linear.y, msg->angular.z);
}

void RobotSerial::actionCallback(const rm_interfaces::msg::Action::SharedPtr msg)
{
    static uint8_t SOF = 0x00;
    Action action{
        msg->max_pitch_w,
        msg->min_pitch_w,
        msg->mode,
        msg->spin,
        msg->patrol,
        msg->robot_aim,
        msg->chasing,
    };
    SOF++;
    sentrySerial.write(0x0502, SOF, action);
    // if (msg->spin==1) {
    //     RCLCPP_INFO(this->get_logger(), "%d %d %d %d", msg->mode, msg->spin, msg->patrol,
    //         msg->chasing);
    // }
}

void RobotSerial::whitelistCallback(const rm_interfaces::msg::Whitelist::SharedPtr msg)
{
    static uint8_t SOF = 0x00;
    Whitelist whitelist{
        msg->robot[0],
        msg->robot[1],
        msg->robot[2],
        msg->robot[3],
        msg->robot[4],
        msg->robot[5],
        msg->robot[6],
        msg->robot[7],
        msg->robot[8],
        msg->robot[9],
        msg->robot[10],
        msg->robot[11],
    };
    SOF++;
    // RCLCPP_INFO(this->get_logger(), "whitelist: %d %d %d %d %d", msg->robot[1], msg->robot[2], msg->robot[3],
    //         msg->robot[4], msg->robot[7]);
    sentrySerial.write(0x0506, SOF, whitelist);
}

void RobotSerial::sentry_cmd_Callback(const rm_interfaces::msg::Decision::SharedPtr msg)
{
    static uint8_t SOF = 0x00;
    sentry_cmd_t sentry_cmd;
    sentry_decision_data_t sentry_decision;

    sentry_cmd.sentry_cmd = sentry_cmd.sentry_cmd & 0b00000000000000000000000000000000;
    sentry_cmd.sentry_cmd = sentry_cmd.sentry_cmd | (((msg->resurrection) & 0b00000000000000000000000000000001) << 0);
    sentry_cmd.sentry_cmd = sentry_cmd.sentry_cmd | (((msg->immediate_resurrection) &
        0b00000000000000000000000000000001) << 1);
    sentry_cmd.sentry_cmd = sentry_cmd.sentry_cmd | (((msg->buy_bullet_at_recovery) &
        0b00000000000000000000011111111111) << 2);
    sentry_cmd.sentry_cmd = sentry_cmd.sentry_cmd | (((msg->buy_bullet_outside) & 0b00000000000000000000000000001111) <<
        13);
    sentry_cmd.sentry_cmd = sentry_cmd.sentry_cmd | (((msg->buy_blood) & 0b00000000000000000000000000001111) << 17);

    sentry_decision.data_cmd_id = 0x0120;
    sentry_decision.sender_id = msg->sender_id;
    sentry_decision.receiver_id = 0x8080;
    sentry_decision.sentry_cmd = sentry_cmd.sentry_cmd;
    SOF++;
    //std::cout<<sentry_cmd.sentry_cmd<<std::endl;
    sentrySerial.write(0x0301, SOF, sentry_decision);
}

void RobotSerial::custom_info_Callback(const rm_interfaces::msg::Info::SharedPtr msg)
{
    static uint8_t SOF = 0x00;
    custom_info_t custom_info;
    custom_info.sender_id = msg->sender_id;
    custom_info.receiver_id = msg->receiver_id;
    const auto& user_data_array = msg->user_data;
    for (size_t i = 0; i < user_data_array.size(); ++i)
    {
        custom_info.user_data[i] = user_data_array[i];
    }
    SOF++;
    // std::cout<<custom_info.user_data<<std::endl;
    sentrySerial.write(0x0308, SOF, custom_info);
}

void RobotSerial::map_data_Callback(const rm_interfaces::msg::Map::SharedPtr msg)
{
    static uint8_t SOF = 0x00;
    map_data_t map_data;
    map_data.intention = msg->intention;
    map_data.start_position_x = msg->start_position_x;
    map_data.start_position_y = msg->start_position_y;
    const auto& delta_x_array = msg->delta_x;
    for (size_t i = 0; i < delta_x_array.size(); ++i)
    {
        map_data.delta_x[i] = delta_x_array[i];
    }
    const auto& delta_y_array = msg->delta_y;
    for (size_t i = 0; i < delta_y_array.size(); ++i)
    {
        map_data.delta_y[i] = delta_y_array[i];
    }
    map_data.sender_id = msg->sender_id;
    SOF++;
    sentrySerial.write(0x0307, SOF, map_data);
}

RobotSerial::RobotSerial() : Node("robot_serial_node")
{
    declare_parameter("/serial_name_sentry", "/dev/sentry_serial");
    sentrySerial = std::move(sentry::SentrySerial(get_parameter("/serial_name_sentry").as_string(), 115200));

    dog_cnt_ = 0;
    this->declare_parameter("dog_threshold", 10);
    this->declare_parameter("frequency", 100.0);

    this->get_parameter("dog_threshold", dog_threshold_);
    this->get_parameter("frequency", frequency);

    period_ns = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / frequency));

    RCLCPP_INFO(this->get_logger(), "robot_serial init success");

    if (0)
    {
        sentrySerial.registerErrorHandle([this](int label, const std::string& text)
        {
            sentry::SentrySerial::error _label;
            _label = (sentry::SentrySerial::error)label;
            std::stringstream _str;
            for (auto c : text)
            {
                _str << std::setw(2) << std::hex << (int)*(uint8_t*)&c << " ";
            }
            _str << std::endl;
            switch (_label)
            {
            case sentry::SentrySerial::lengthNotMatch:
                RCLCPP_ERROR_STREAM(get_logger(), "sentry_lengthNotMatch");
                RCLCPP_ERROR_STREAM(get_logger(), _str.str());
            case sentry::SentrySerial::rxLessThanLength:
                RCLCPP_ERROR_STREAM(get_logger(), "sentry_rxLessThanLength");
                break;
            case sentry::SentrySerial::crcError:
                RCLCPP_ERROR_STREAM(get_logger(), "sentry_crc8Error");
                RCLCPP_ERROR_STREAM(get_logger(), _str.str());
                break;
            case sentry::SentrySerial::crc16Error:
                RCLCPP_ERROR_STREAM(get_logger(), "sentry_crc16Error");
                RCLCPP_ERROR_STREAM(get_logger(), _str.str());
                break;
            default:
                return;
            }
        });
    }

    //串口回调函数，接收下位机转发的消息
    sentrySerial.registerCallback(0x0606, [this](const data_received_packag_200hz_t& msg)
    {
        rm_interfaces::msg::Hp _Hp; //0x0003
        _Hp.red_1_robot_hp = msg.robot_HP.red_1_robot_HP;
        _Hp.red_2_robot_hp = msg.robot_HP.red_2_robot_HP;
        _Hp.red_3_robot_hp = msg.robot_HP.red_3_robot_HP;
        _Hp.red_4_robot_hp = msg.robot_HP.red_4_robot_HP;
        _Hp.red_7_robot_hp = msg.robot_HP.red_7_robot_HP;
        _Hp.red_outpost_hp = msg.robot_HP.red_outpost_HP;
        _Hp.red_base_hp = msg.robot_HP.red_base_HP;
        _Hp.blue_1_robot_hp = msg.robot_HP.blue_1_robot_HP;
        _Hp.blue_2_robot_hp = msg.robot_HP.blue_2_robot_HP;
        _Hp.blue_3_robot_hp = msg.robot_HP.blue_3_robot_HP;
        _Hp.blue_4_robot_hp = msg.robot_HP.blue_4_robot_HP;
        _Hp.blue_7_robot_hp = msg.robot_HP.blue_7_robot_HP;
        _Hp.blue_outpost_hp = msg.robot_HP.blue_outpost_HP;
        _Hp.blue_base_hp = msg.robot_HP.blue_base_HP;
        HpPublisher->publish(_Hp);

        rm_interfaces::msg::HurtData _HurtData;//0x0206
        _HurtData.armor_id = msg.hurt_data.armor_id;
        _HurtData.hp_deduction_reason = msg.hurt_data.hp_deduction_reason;
        HurtDataPublisher->publish(_HurtData);

        rm_interfaces::msg::Projectileallowance _Projectileallowance; //0x0208
        _Projectileallowance.projectile_allowance_17mm = msg.projectile_allowance.projectile_allowance_17mm;
        _Projectileallowance.remaining_gold_coin = msg.projectile_allowance.remaining_gold_coin;
        ProjectileallowancePublisher->publish(_Projectileallowance);

        rm_interfaces::msg::Rfidstatus _Rfidstatus; //0x0209
        _Rfidstatus.base = msg.rfid_status.base;
        _Rfidstatus.our_central_plateau = msg.rfid_status.our_central_plateau;
        _Rfidstatus.his_central_plateau = msg.rfid_status.his_central_plateau;
        _Rfidstatus.our_trapezoidal_heights = msg.rfid_status.our_trapezoidal_heights;
        _Rfidstatus.his_trapezoidal_heights = msg.rfid_status.his_trapezoidal_heights;
        _Rfidstatus.our_cross_feipo_front = msg.rfid_status.our_cross_feipo_front;
        _Rfidstatus.our_cross_feipo_back = msg.rfid_status.our_cross_feipo_back;
        _Rfidstatus.his_cross_feipo_front = msg.rfid_status.his_cross_feipo_front;
        _Rfidstatus.his_cross_feipo_back = msg.rfid_status.his_cross_feipo_back;
        _Rfidstatus.our_cross_central_below = msg.rfid_status.our_cross_central_below;
        _Rfidstatus.our_cross_central_up = msg.rfid_status.our_cross_central_up;
        _Rfidstatus.his_cross_central_below = msg.rfid_status.his_cross_central_below;
        _Rfidstatus.his_cross_central_up = msg.rfid_status.his_cross_central_up;
        _Rfidstatus.our_cross_road_below = msg.rfid_status.our_cross_road_below;
        _Rfidstatus.our_cross_road_up = msg.rfid_status.our_cross_road_up;
        _Rfidstatus.his_cross_road_below = msg.rfid_status.his_cross_road_below;
        _Rfidstatus.his_cross_road_up = msg.rfid_status.his_cross_road_up;
        _Rfidstatus.our_fort = msg.rfid_status.our_fort;
        _Rfidstatus.outpost = msg.rfid_status.outpost;
        _Rfidstatus.inside_recovery = msg.rfid_status.inside_recovery;
        _Rfidstatus.outside_recovery = msg.rfid_status.outside_recovery;
        _Rfidstatus.our_big_resources = msg.rfid_status.our_big_resources;
        _Rfidstatus.his_big_resources = msg.rfid_status.his_big_resources;
        _Rfidstatus.center = msg.rfid_status.center;
        _Rfidstatus.his_fort = msg.rfid_status.his_fort;
        RfidstatusPublisher->publish(_Rfidstatus);

        dog_cnt_ = 0; //串口看门狗清零

        rm_interfaces::msg::AngleError _AngleError; //0x0601
        _AngleError.angle_error = msg.angle_error.angle_error;
        _AngleError.gimbal_error = msg.angle_error.gimbal_error;
        AngleErrorPublisher->publish(_AngleError);
    });
    sentrySerial.registerCallback(0x0607, [this](const data_received_packag_1hz_t& msg)
    {
        // RCLCPP_INFO(get_logger(), "received package");
        rm_interfaces::msg::Gamestatus _Gamestatus; //0x0001
        _Gamestatus.game_progress = msg.game_status.game_progress;
        _Gamestatus.stage_remain_time = msg.game_status.stage_remain_time;
        GamestatusPublisher->publish(_Gamestatus);

        rm_interfaces::msg::Event _Event; //0x0101
        _Event.inside_recovery = msg.event_data.inside_recovery;
        _Event.outside_recovery = msg.event_data.outside_recovery;
        _Event.recovery_rmul = msg.event_data.recovery_rmul;
        _Event.small_energy_organ_status = msg.event_data.small_energy_organ_status;
        _Event.big_energy_organ_status = msg.event_data.big_energy_organ_status;
        _Event.central_plateau = msg.event_data.central_plateau;
        _Event.trapezoidal_heights = msg.event_data.trapezoidal_heights;
        _Event.time_of_dart_hit = msg.event_data.time_of_dart_hit;
        _Event.target_of_dart_hit = msg.event_data.target_of_dart_hit;
        _Event.center_rmul = msg.event_data.center_rmul;
        _Event.our_fort = msg.event_data.our_fort;
        EventPublisher->publish(_Event);

        rm_interfaces::msg::Dartinfo _Dartinfo; //0x0105
        _Dartinfo.dart_count_aim = msg.dart_info.dart_count_aim;
        DartinfoPublisher->publish(_Dartinfo);

        rm_interfaces::msg::Robotp _Robotp; //0x0203
        _Robotp.x = msg.robot_pos.x;
        _Robotp.y = msg.robot_pos.y;
        _Robotp.angle = msg.robot_pos.angle;
        RobotpPublisher->publish(_Robotp);

        rm_interfaces::msg::Buff _Buff; //0x0204
        _Buff.remaining_energy = msg.buff.remaining_energy;
        _Buff.defence_buff = msg.buff.defence_buff;
        BuffPublisher->publish(_Buff);

        rm_interfaces::msg::Robotposition _Robotposition; //0x020B
        _Robotposition.hero_x = msg.ground_robot_position.hero_x;
        _Robotposition.hero_y = msg.ground_robot_position.hero_y;
        _Robotposition.engineer_x = msg.ground_robot_position.engineer_x;
        _Robotposition.engineer_y = msg.ground_robot_position.engineer_y;
        _Robotposition.standard_3_x = msg.ground_robot_position.standard_3_x;
        _Robotposition.standard_3_y = msg.ground_robot_position.standard_3_y;
        _Robotposition.standard_4_x = msg.ground_robot_position.standard_4_x;
        _Robotposition.standard_4_y = msg.ground_robot_position.standard_4_y;
        RobotpositionPublisher->publish(_Robotposition);

        rm_interfaces::msg::Sentryinfo _Sentryinfo; //0x020D
        _Sentryinfo.amount_of_get_ammunition = msg.sentry_info.amount_of_get_ammunition;
        _Sentryinfo.number_of_get_ammunition = msg.sentry_info.number_of_get_ammunition;
        _Sentryinfo.number_of_get_blood = msg.sentry_info.number_of_get_blood;
        _Sentryinfo.confirm_free_resurrection = msg.sentry_info.confirm_free_resurrection;
        _Sentryinfo.confirm_resurrection_immediately = msg.sentry_info.confirm_resurrection_immediately;
        _Sentryinfo.cost_of_resurrection = msg.sentry_info.cost_of_resurrection;
        _Sentryinfo.if_out_fight = msg.sentry_info.if_out_fight;
        _Sentryinfo.projectile_allowance_17mm_of_team = msg.sentry_info.projectile_allowance_17mm_of_team;
        SentryinfoPublisher->publish(_Sentryinfo);

        rm_interfaces::msg::Lidarstation _Lidarstation;
       _Lidarstation.attack_enhance = msg.lidarstation_data.attack_enhance;
       for(int i = 0; i < 5; i++)
       {
           rm_interfaces::msg::Lidarposition Lidarposition;
           Lidarposition.x = msg.lidarstation_data.robot_x[i];
           Lidarposition.y = msg.lidarstation_data.robot_y[i];
           Lidarposition.z = msg.lidarstation_data.robot_z[i];
           _Lidarstation.lidarpositions[i] = Lidarposition;
       }
       LidarstationPublisher->publish(_Lidarstation);

        rm_interfaces::msg::Mapcommand _Mapcommand; //0x0303
        _Mapcommand.target_position_x = msg.map_command.target_position_x;
        _Mapcommand.target_position_y = msg.map_command.target_position_y;
        _Mapcommand.cmd_keyboard = msg.map_command.cmd_keyboard;
        MapcommandPublisher->publish(_Mapcommand);

        dog_cnt_ = 0; //串口看门狗清零
    });

    BuffPublisher = create_publisher<rm_interfaces::msg::Buff>("/robot/buff", 1);
    DartinfoPublisher = create_publisher<rm_interfaces::msg::Dartinfo>("/robot/dartinfo", 1);
    EventPublisher = create_publisher<rm_interfaces::msg::Event>("/robot/event", 1);
    GamestatusPublisher = create_publisher<rm_interfaces::msg::Gamestatus>("/robot/gamestatus", 1);
    HpPublisher = create_publisher<rm_interfaces::msg::Hp>("/robot/hp", 1);
    InteractionPublisher = create_publisher<rm_interfaces::msg::Interaction>("/robot/interaction", 1);
    MapcommandPublisher = create_publisher<rm_interfaces::msg::Mapcommand>("/robot/mapcommand", 1);
    HurtDataPublisher  = create_publisher<rm_interfaces::msg::HurtData>("/robot/hurtdata", 1);
    ProjectileallowancePublisher = create_publisher<rm_interfaces::msg::Projectileallowance>(
        "/robot/projectileallowance",
        1
    );
    RfidstatusPublisher = create_publisher<rm_interfaces::msg::Rfidstatus>("/robot/rfidstatus", 1);
    RobotpPublisher = create_publisher<rm_interfaces::msg::Robotp>("/robot/robotp", 1);
    RobotpositionPublisher = create_publisher<rm_interfaces::msg::Robotposition>("/robot/robotposition", 1);
    RobotstatusPublisher = create_publisher<rm_interfaces::msg::Robotstatus>("/robot/robotstatus", 1);
    SentryinfoPublisher = create_publisher<rm_interfaces::msg::Sentryinfo>("/robot/sentryinfo", 1);
    LidarstationPublisher = create_publisher<rm_interfaces::msg::Lidarstation>("/robot/lidarstation", 1);
    AngleErrorPublisher = create_publisher<rm_interfaces::msg::AngleError>("/robot/angleerror", 1);

    VelocitySubscription = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1,
                                                                          std::bind(&RobotSerial::velocityCallback,
                                                                              this,
                                                                              std::placeholders::_1));
    DecisionSubscription = create_subscription<rm_interfaces::msg::Decision>("/robot/decision", 1,
                                                                             std::bind(
                                                                                 &RobotSerial::sentry_cmd_Callback,
                                                                                 this,
                                                                                 std::placeholders::_1));
    ActionSubscription = create_subscription<rm_interfaces::msg::Action>("/robot/action", 1,
                                                                         std::bind(&RobotSerial::actionCallback, this,
                                                                             std::placeholders::_1));
    InfoSubscription = create_subscription<rm_interfaces::msg::Info>("/robot/info", 1,
                                                                     std::bind(&RobotSerial::custom_info_Callback, this,
                                                                               std::placeholders::_1));
    MapSubscription = create_subscription<rm_interfaces::msg::Map>("/robot/map", 1,
                                                                   std::bind(&RobotSerial::map_data_Callback, this,
                                                                             std::placeholders::_1));
    WhitelistSubscription = create_subscription<rm_interfaces::msg::Whitelist>("/robot/whitelist", 1,
                                                                               std::bind(
                                                                                   &RobotSerial::whitelistCallback,
                                                                                   this, std::placeholders::_1));
    timer_ = this->create_wall_timer(period_ns, std::bind(&RobotSerial::timer_callback, this));

    odom_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    sentrySerial.spin(true);
}

void RobotSerial::timer_callback()
{
    dog_cnt_++;
    if (dog_cnt_ > dog_threshold_)
    {
        RCLCPP_ERROR(this->get_logger(), "长时间未收到C板数据，重启串口");
        throw;
    }
}
