//
// Demonstrates how to add and fly Waypoint missions using the MAVSDK.
//

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mission_raw/mission_raw.h>
#include <mavsdk/plugins/offboard/offboard.h>

// reexport mavlink headers using this plugin
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/param/param.h>



#include <chrono>
#include <functional>
#include <future>
#include <iostream>
#include <ratio>
#include <thread>

using namespace mavsdk;
using std::chrono::seconds;
using std::chrono::high_resolution_clock;
using std::this_thread::sleep_for;

static void wait_armable(bool& is_armable)
{
    while (is_armable != true) {
        std::cout << "Vehicle is getting ready to arm\n";
        sleep_for(seconds(1));
    }
}


MissionRaw::MissionItem make_mission_item_wp(
    float latitude_deg1e7,
    float longitude_deg1e7,
    int32_t altitude_m,
    float param1,
    MAV_FRAME frame,
    MAV_CMD command,
    float p2 = 0,
    float p3 = 0)
{
    // WARNING this is done in consideration of CLEAN!! mission
    static uint32_t seq_num = 0;
    MissionRaw::MissionItem new_item{};
    new_item.seq = seq_num;
    new_item.frame = static_cast<uint32_t>(frame);
    new_item.command = static_cast<uint32_t>(command);
    new_item.param1 = param1;
    new_item.param2 = p2;
    new_item.param3 = p3;   
    new_item.x = latitude_deg1e7 * 1e7;
    new_item.y = longitude_deg1e7 * 1e7;
    new_item.z = altitude_m;
    new_item.mission_type = MAV_MISSION_TYPE_MISSION;
    new_item.autocontinue = 1;

    if (seq_num == 1) {
        new_item.current = 1;
    }

    seq_num++;

    return new_item;
}


bool create_missionPlan(std::vector<mavsdk::MissionRaw::MissionItem>& mission_plan, float lat_deg_home = -35.359833, float lon_deg_home = 149.164703){


    // in case of ardupilot we want to set lat lon to 0, to use current position as takeoff position
    mission_plan.push_back(make_mission_item_wp( //0
        lat_deg_home, // lat home
        lon_deg_home, // lon home
        100, // alt home
        0,
        MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MAV_CMD_NAV_WAYPOINT));

    mission_plan.push_back(make_mission_item_wp( // 1 takeoff
        -35.359833, // lat
        149.164703, // lon
        41.03,
        15,
        MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MAV_CMD_NAV_TAKEOFF));

    // // setup speed during mission execution
    // mission_plan.push_back(make_mission_item_wp(
    //     0, 0, 0, 0, MAV_FRAME_GLOBAL_RELATIVE_ALT, MAV_CMD_DO_CHANGE_SPEED, 9.35f, -1.0f));

    mission_plan.push_back(make_mission_item_wp( //2
        -35.359585,
        149.161392,
        100.00,
        0,
        MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MAV_CMD_NAV_WAYPOINT));

    mission_plan.push_back(make_mission_item_wp( //3
        -35.366463,
        149.162231,
        100.00,
        0,
        MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MAV_CMD_NAV_WAYPOINT));


    mission_plan.push_back(make_mission_item_wp( //4
        -35.366131,
        149.164581,
        100.00,
        0,
        MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MAV_CMD_NAV_WAYPOINT));
 
    mission_plan.push_back(make_mission_item_wp( //5
        -35.359272,
        149.163757,
        100.00,
        0,
        MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MAV_CMD_NAV_WAYPOINT));

    mission_plan.push_back(make_mission_item_wp( //6
        -35.366131, // wont do anything
        149.164581, // wont do anything
        100.00,     // wont do anything
        SPEED_TYPE_AIRSPEED,          // param 1
        MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MAV_CMD_DO_CHANGE_SPEED,
        6) // param 2 - 6m/s
    );

    mission_plan.push_back(make_mission_item_wp( //7
        -35.359272,
        149.163757,
        100.00,
        0,
        MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MAV_CMD_NAV_WAYPOINT));

    mission_plan.push_back(make_mission_item_wp( //8
        -35.3608654,
        149.1648848,
        41.00,
        0,
        MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MAV_CMD_NAV_WAYPOINT));

    mission_plan.push_back(make_mission_item_wp( //9
        lat_deg_home,
        lon_deg_home,
        0.00,
        1, //m Minimum abort altitude
        MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MAV_CMD_NAV_LAND,
        PRECISION_LAND_MODE_OPPORTUNISTIC));

    return true;
}






int main(int argc, char** argv)
{

    Mavsdk mavsdk{Mavsdk::Configuration{Mavsdk::ComponentType::GroundStation}};
    ConnectionResult connection_result = mavsdk.add_any_connection(argv[1]);

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return 1;
    }

     auto system = mavsdk.first_autopilot(3.0);
    if (!system) {
        std::cerr << "Timed out waiting for system\n";
        return 1;
    }

    auto mission_raw = MissionRaw(system.value());
    auto action = Action(system.value());
    auto telemetry = Telemetry{system.value()};
    bool is_armable = false;

    {
        auto set_rate_result = telemetry.set_rate_position(7.0);
        if (set_rate_result != Telemetry::Result::Success) {
            std::cerr << "Setting position rate failed: " << set_rate_result << '\n';
            return 1;
        }
        else{
            std::cout << "Setting position rate success" << std::endl;
        }
    }

    /*
        This one is neccesarry for ArduPilot because this subscription request
        SYS_STATUS message https://mavlink.io/en/messages/common.html#SYS_STATUS
        which is used for is_armable flag

        this is bug in case of ardupilot. See https://github.com/mavlink/MAVSDK/issues/1996
    */
    {
        auto set_rate_result = telemetry.set_rate_battery(7.0);
        if (set_rate_result != Telemetry::Result::Success) {
            std::cerr << "Setting bat rate failed: " << set_rate_result << '\n';
            return 1;
        }
    }

    auto handle = telemetry.subscribe_health([&is_armable](Telemetry::Health h) { is_armable = h.is_armable; });

// debug timings for position messages
#if 0
    telemetry.subscribe_position([](Telemetry::Position p) {
        static auto tstatic = std::chrono::high_resolution_clock::now();
        auto t = high_resolution_clock::now();
        std::cout << "Pos HZ: " << 1e9/std::chrono::duration_cast<std::chrono::nanoseconds>(t-tstatic).count() << std::endl;
        tstatic = std::chrono::high_resolution_clock::now();
    });
#endif

    telemetry.subscribe_flight_mode([](Telemetry::FlightMode f) { std::cout << "Flight mode: " << f << " - " << static_cast<int>(f) << std::endl; });

#if 0
    std::cout << "subscribing battery status" << std::endl;
    telemetry.subscribe_battery([](Telemetry::Battery b) { std::cout << b << std::endl; });
#endif

//     mission_raw.subscribe_mission_progress([&action](MissionRaw::MissionProgress progress) {
//         std::cout << progress << std::endl;
// // enable this if you want to pause mission halfway
// #if 0
//         // if (3 == progress.current) {
//         //     action.hold();
//         // }
// #endif
//     });

    /*
     * wait for armable flag, this is neccesarry,
     * because we should allow autopilot to initialize all its systems
     */
    std::cout << "Waiting for system to be armable...\n";
    wait_armable(is_armable);

    auto clear_result = mission_raw.clear_mission();
    if (clear_result != MissionRaw::Result::Success) {
        std::cout << "clear failed" << std::endl;
        return 1;
    }

    auto download_result = mission_raw.download_mission();
    if (download_result.first != MissionRaw::Result::Success) {
        std::cout << "Download failed" << std::endl;
        return 1;
    }

    // first point in case of ardupilot is always home
    auto mission_plan = download_result.second;
    
    MissionRaw::MissionItem home_point = mission_plan[0];

    std::cout << "Home point: " << home_point << std::endl;
    std::cout << "-----------------------------------------------" << std::endl;

    mission_plan.clear();

    // going relative alt mission so we dont care about altitude
    auto lat_deg_home = home_point.x * 1e-7;
    auto lon_deg_home = home_point.y * 1e-7;
    create_missionPlan(mission_plan);

    mission_plan.back().autocontinue = 1;

    for (const auto& item : mission_plan) {
        std::cout << "seq: " << (int)item.seq << '\n';
        // std::cout << "MissionRaw: " << item << "\n\n";
    }

    auto upload_result = mission_raw.upload_mission(mission_plan);
    if (upload_result != MissionRaw::Result::Success) {
        std::cout << "upload failed" << std::endl;
        std::cout << "upload result: " << upload_result << std::endl;
        return 1;
    }

    mission_raw.set_current_mission_item(0);

    // start mission, this set autopilot auto mode.
    // ignore result, we dont care for now
    auto start_result = mission_raw.start_mission();
    if (start_result != MissionRaw::Result::Success) {
        std::cout << "start failed" << std::endl;
    }else{
        std::cout << "Mission started" << std::endl;
    }

    // // wait for two secs to be shure is_armable flag updated
    // sleep_for(seconds(2));
    // while (true) {
    //     /*
    //      * Need to wait for is_armable flag.
    //      * Ardupilot sets is_aramble flag in all non auto controlled modes earlier,
    //      * because it doesnt check position consistency
    //      * In auto controlled modes, we should wait a bit more so we recheck it again
    //      */
    //     wait_armable(is_armable);
    //     std::cout << "Arming...\n";
    //     const Action::Result arm_result = action.arm();
    //     switch (arm_result) {
    //         case Action::Result::Success: { // somehow this version of mavsdk sometime reset mode on
    //             std::cout << "Arming success" << std::endl; // arm, so we explicitly set mode to auto again
    //             break;
    //         }
    //         case Action::Result::CommandDenied:
    //             // arming denied for some reason probably we are not ready yet or some serios error
    //             // in autopilot occured
    //             std::cerr << "Arming Denied: " << '\n';
    //             sleep_for(seconds(1));
    //             continue;
    //         default:
    //             std::cerr << "Arming failed: " << arm_result << '\n';
    //             return 1;
    //     }
    //     break;
    // }

    std::cout << "Arming...\n";
    const Action::Result arm_result = action.arm();
    if (arm_result != Action::Result::Success) {
        std::cerr << "Arming failed: " << arm_result << '\n';
        return 1;
    }
    std::cout << "Armed.\n";

    auto mavPass = MavlinkPassthrough(system.value());
     std::cout << "sending command to set speed\n";
    auto command_mode = MavlinkPassthrough::CommandLong{};
    command_mode.command = MAV_CMD_DO_CHANGE_SPEED;
    command_mode.target_sysid = system.value()->get_system_id();
    command_mode.target_compid = MAV_COMP_ID_AUTOPILOT1; //system.value()->component_ids().front(); // assuming first component is autopilot
    command_mode.param1 = SPEED_TYPE_AIRSPEED;
    command_mode.param2 = 7;
    command_mode.param3 = -1; // -1 throttle no change
    auto sendres = mavPass.send_command_long(command_mode);
    // wait some time to allow copter to gain altitude, and only then start in_air check
    sleep_for(seconds(10));


    std::cout << "sending command to set mode to GUIDED_ARMED\n"; 

    // auto command_mode = MavlinkPassthrough::CommandLong{};
    // command_mode.command = MAV_CMD_DO_SET_MODE;
    // command_mode.target_sysid = system.value()->get_system_id();
    // command_mode.target_compid = system.value()->component_ids().front(); // assuming first component is autopilot
    // command_mode.param1 = MAV_MODE_GUIDED_ARMED;
    // mavPass.send_command_long(command_mode);



    auto res = action.goto_location(lat_deg_home+0.0011,
                         lon_deg_home+0.0011,
                         home_point.z + 60,
                         0.0);

    if(res != Action::Result::Success){
        std::cerr << "goto_location failed: " << res << '\n';
    }

    res = action.set_maximum_speed(30);
    if(res != Action::Result::Success){
        std::cerr << "set_maximum_speed failed: " << res << '\n';
    }


    auto result = action.get_maximum_speed(); 
    if(result.first != Action::Result::Success){
        std::cerr << "get_maximum_speed failed: " << result.first << '\n';
    }else{
        std::cout << "Maximum speed: " << result.second << '\n';
    }

    res = action.set_current_speed(4.0); // groundspeed
    
    if(res != Action::Result::Success){
        std::cerr << "set_current_speed failed: " << res << '\n';
    } else
    {
        std::cout << "set_current_speed successfully set to 4\n";
    }

    auto resp = mavPass.get_param_int("AIRSPEED_MAX", std::nullopt, false);

    if (resp.first != MavlinkPassthrough::Result::Success) {
        std::cerr << "get_param_int failed: " << resp.first << '\n';
    } else {
        std::cout << "AIRSPEED_MAX: " << resp.second << '\n';
    }
    
    auto resp2 = mavPass.get_param_int("AIRSPEED_MIN", std::nullopt, false);
    if (resp2.first != MavlinkPassthrough::Result::Success) {
        std::cerr << "get_param_int failed: " << resp2.first << '\n';
    } else {
        std::cout << "AIRSPEED_MIN: " << resp2.second << '\n';
    }


    std::cout << "Getting AIRSPEED with PARAM --------------------\n";
    auto param = Param(system.value());

    // Check for our custom param we have set in the server thread
    auto par_res = param.get_param_int("AIRSPEED_MAX");
    if (par_res.first == mavsdk::Param::Result::Success) {
        std::cout << "Found Param (int) AIRSPEED_MAX: " << par_res.second << std::endl;
    } else {
        std::cout << "Param (int) AIRSPEED_MAX not found " << par_res.first << std::endl;
    }

    auto parRes = param.set_param_int("AIRSPEED_MIN", 5);

    if (parRes == mavsdk::Param::Result::Success) {
        std::cout << "set Param (int) AIRSPEED_MIN: SET" <<  std::endl;
    } else {
        std::cout << "Param (int) AIRSPEED_MIN not set " << parRes << std::endl;
    }

    resp2 = mavPass.get_param_int("AIRSPEED_MIN", std::nullopt, false);
    if (resp2.first != MavlinkPassthrough::Result::Success) {
        std::cerr << "get_param_int failed: " << resp2.first << '\n';
    } else {
        std::cout << "AIRSPEED_MIN: " << resp2.second << '\n';
    }

    
    // auto all = param.get_all_params();
    // std::cout << "Printing all parameters: " << std::endl;
    // std::cout <<  all << "\n ------------------- " << std::endl;




    std::cout << "sending command to set speed\n";
    command_mode = MavlinkPassthrough::CommandLong{};
    command_mode.command = MAV_CMD_DO_CHANGE_SPEED;
    command_mode.target_sysid = system.value()->get_system_id();
    command_mode.target_compid = MAV_COMP_ID_AUTOPILOT1; //system.value()->component_ids().front(); // assuming first component is autopilot
    command_mode.param1 = SPEED_TYPE_AIRSPEED;
    command_mode.param2 = 7;
    command_mode.param3 = -1; // -1 throttle no change
    sendres = mavPass.send_command_long(command_mode);
    if(sendres != MavlinkPassthrough::Result::Success) {
        std::cerr << "send_command_long failed: " << sendres << '\n';
        auto comps = system.value()->component_ids();
        for(auto comp : comps){
            std::cout << "Component: " << static_cast<int>(comp) << " / " << static_cast<int>(command_mode.target_compid) << std::endl;
        }
        std::cout << "component size is " << comps.size() << std::endl;
        

    } else {
        std::cout << "send_command_long success\n";
    }


    // Check if vehicle is still in air
    while (telemetry.in_air()) {
        sleep_for(seconds(1));
    }
    std::cout << "Landed!\n";

    // We are relying on auto-disarming but let's keep watching the telemetry for a bit longer.
    sleep_for(seconds(3));
    std::cout << "Finished...\n";
}
